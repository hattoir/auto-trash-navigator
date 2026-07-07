#!/usr/bin/env python3

import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from action_msgs.msg import GoalStatus

# Waypoint Coordinates Design:
# Wall boundary: x, y in [-4, 4]. 1.2m offset from walls -> x, y in [-2.8, 2.8]
# Obstacles at: (2, 2), (-2, -2), (-2, 2). 1.2m offset -> distance to obstacle centers >= 1.2m
#
# Chosen Points:
# 1. Waypoint 1: (2.5, -2.5) -> Safe area in Quadrant IV (no obstacles). Wall offset is 1.5m.
# 2. Waypoint 2: (0.0, 2.5)  -> Centered between obstacles (2,2) and (-2,2). Distance to both is ~2.06m. Wall offset is 1.5m.
# 3. Waypoint 3: (-2.5, -0.5) -> Offset from obstacle (-2, 2) by ~2.55m and (-2, -2) by ~1.58m. Wall offset is 1.5m.
#
# These three points form a large triangle avoiding all walls and obstacles.

WAYPOINTS_DATA = [
    {"x": 2.5, "y": -2.5, "yaw": 0.0},
    {"x": 0.0, "y": 2.5, "yaw": 1.57},
    {"x": -2.5, "y": -0.5, "yaw": 3.14}
]

class InitialPoseChecker(Node):
    def __init__(self):
        super().__init__('initial_pose_checker')
        self.received = False
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.cb,
            10
        )

    def cb(self, msg):
        self.received = True

def check_amcl_initialized():
    node = InitialPoseChecker()
    start_time = time.time()
    # Wait up to 2 seconds to check if amcl_pose is published
    while time.time() - start_time < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.received:
            break
    node.destroy_node()
    return node.received

def main():
    rclpy.init()

    # Create navigator
    navigator = BasicNavigator()

    # Check if AMCL is already initialized to avoid double setting
    amcl_initialized = check_amcl_initialized()
    if not amcl_initialized:
        navigator.get_logger().info("AMCL not initialized. Setting initial pose to (0,0)...")
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = navigator.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        init_pose.pose.orientation.w = 1.0
        navigator.setInitialPose(init_pose)
    else:
        navigator.get_logger().info("AMCL is already initialized. Skipping setInitialPose.")

    # Wait for Nav2 to activate
    navigator.get_logger().info("Waiting for Nav2 to become active...")
    navigator.waitUntilNav2Active()
    navigator.get_logger().info("Nav2 is fully active. Starting patrol.")

    consecutive_failures = [0] * len(WAYPOINTS_DATA)
    loop_count = 0

    try:
        while rclpy.ok():
            loop_count += 1
            navigator.get_logger().info(f"========== Starting Patrol Loop {loop_count} ==========")
            start_time = time.time()

            # Build list of active waypoints (exclude those with >= 3 failures)
            active_waypoints = []
            active_indices = []
            for i, wp in enumerate(WAYPOINTS_DATA):
                if consecutive_failures[i] < 3:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = navigator.get_clock().now().to_msg()
                    pose.pose.position.x = wp["x"]
                    pose.pose.position.y = wp["y"]
                    
                    # Convert yaw to quaternion
                    q_z = math.sin(wp["yaw"] / 2.0)
                    q_w = math.cos(wp["yaw"] / 2.0)
                    pose.pose.orientation.z = q_z
                    pose.pose.orientation.w = q_w
                    
                    active_waypoints.append(pose)
                    active_indices.append(i)

            if not active_waypoints:
                navigator.get_logger().error("All waypoints have failed 3 consecutive times. Exiting.")
                break

            navigator.get_logger().info(f"Active waypoints in this loop: {[idx + 1 for idx in active_indices]}")

            # Send waypoints to follow
            navigator.followWaypoints(active_waypoints)

            # Monitor progress
            last_waypoint_index = -1
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    current_wp = feedback.current_waypoint
                    if current_wp != last_waypoint_index:
                        global_wp_index = active_indices[current_wp]
                        navigator.get_logger().info(
                            f"Navigating to Waypoint {global_wp_index + 1} (x: {WAYPOINTS_DATA[global_wp_index]['x']}, y: {WAYPOINTS_DATA[global_wp_index]['y']})"
                        )
                        last_waypoint_index = current_wp
                time.sleep(1.0)

            # Check loop results
            result = navigator.getResult()
            end_time = time.time()
            duration = end_time - start_time

            # Determine which waypoints failed (missed)
            missed_indices = []
            action_result = navigator.result_future.result().result
            if action_result and hasattr(action_result, 'missed_waypoints'):
                missed_active_indices = action_result.missed_waypoints
                missed_indices = [active_indices[mw.index] for mw in missed_active_indices]

            if result == TaskResult.SUCCEEDED:
                navigator.get_logger().info(f"Loop {loop_count} completed successfully in {duration:.2f} seconds.")
                # Reset failure counters for all active waypoints
                for idx in active_indices:
                    consecutive_failures[idx] = 0
            else:
                navigator.get_logger().warn(
                    f"Loop {loop_count} finished with failures in {duration:.2f} seconds. Result code: {result}"
                )
                # Handle failures
                for idx in active_indices:
                    if idx in missed_indices:
                        consecutive_failures[idx] += 1
                        navigator.get_logger().error(
                            f"Waypoint {idx + 1} failed to reach! Consecutive failures: {consecutive_failures[idx]}"
                        )
                        if consecutive_failures[idx] >= 3:
                            navigator.get_logger().error(
                                f"Waypoint {idx + 1} failed 3 times consecutively. Safety shutdown triggered."
                            )
                            navigator.cancelTask()
                            rclpy.shutdown()
                            sys.exit(1)
                    else:
                        consecutive_failures[idx] = 0

            time.sleep(2.0)

    except KeyboardInterrupt:
        navigator.get_logger().info("KeyboardInterrupt received. Canceling task and shutting down...")
        navigator.cancelTask()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
