#!/usr/bin/env python3
"""Phase 3: 巡回・ゴミ検出・接近・ダミー回収・巡回再開ノード

状態遷移:
  PATROL -> APPROACH -> COLLECT -> PATROL
"""
import math
import time
import threading
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.srv import GetPlan
import tf2_ros

WAYPOINTS = [
    (2.4, -2.4),
    (0.0, 2.4),
    (-2.4, 0.0),
]
TRASH_COORDINATES = {
    1: (2.2, -1.8),
    2: (0.2, 2.2),
    3: (-2.2, 0.2),
}
APPROACH_DISTANCE = 0.30
MAX_CONSECUTIVE_FAILURES = 3
GOAL_TIMEOUT_SEC = 120.0

def make_pose(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p

def get_robot_pose(navigator):
    try:
        # Lookup transform from 'map' to 'base_footprint'
        transform = navigator.tf_buffer.lookup_transform(
            'map',
            'base_footprint',
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5)
        )
        rx = transform.transform.translation.x
        ry = transform.transform.translation.y
        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        ryaw = math.atan2(siny_cosp, cosy_cosp)
        return rx, ry, ryaw
    except Exception as e:
        navigator.get_logger().warn(f"Failed to lookup robot pose: {e}")
        return None, None, None

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Initialize TF listener on the navigator node
    navigator.tf_buffer = tf2_ros.Buffer()
    navigator.tf_listener = tf2_ros.TransformListener(navigator.tf_buffer, navigator)
    
    # State machine variables
    navigator.state = 'PATROL'
    navigator.current_wp_idx = 0
    navigator.detected_trash_queue = []
    navigator.processed_trash = []
    navigator.blacklisted_trash = []
    navigator.lock = threading.Lock()
    navigator.new_trash_event = threading.Event()
    navigator.current_target_trash = None
    navigator.pick_client = navigator.create_client(GetPlan, '/pick_trash')
    
    # Subscriber to detected trash
    def trash_callback(msg):
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        
        with navigator.lock:
            # ONLY detect and queue trash when in PATROL state
            if navigator.state != 'PATROL':
                return
                
            # Filter out detections too close to the robot base (e.g. robot arm or body parts)
            rx, ry, ryaw = get_robot_pose(navigator)
            if rx is not None:
                dist_to_robot = math.sqrt((tx - rx)**2 + (ty - ry)**2)
                if dist_to_robot < 0.60:
                    return
            
            # Filter out wall corner false detections (true papers are at least 0.5m away from +/- 4.0m walls)
            if abs(tx) > 3.48 or abs(ty) > 3.48:
                return
                
            # Filter out obstacle false detections (true papers are at least 0.5m away from obstacles)
            obstacle_centers = [
                (2.0, 2.0),
                (-2.0, -2.0),
                (-2.0, 2.0)
            ]
            is_near_obstacle = False
            for ox, oy in obstacle_centers:
                dist_to_obs = math.sqrt((tx - ox)**2 + (ty - oy)**2)
                if dist_to_obs < 0.45:
                    is_near_obstacle = True
                    break
            if is_near_obstacle:
                return
            
            # Check processed list
            for px, py, pz in navigator.processed_trash:
                dist = math.sqrt((tx - px)**2 + (ty - py)**2 + (tz - pz)**2)
                if dist < 0.3:
                    return
            # Check blacklist
            for bx, by, bz in navigator.blacklisted_trash:
                dist = math.sqrt((tx - bx)**2 + (ty - by)**2 + (tz - bz)**2)
                if dist < 0.3:
                    return
            # Check queue
            for qx, qy, qz in navigator.detected_trash_queue:
                dist = math.sqrt((tx - qx)**2 + (ty - qy)**2 + (tz - qz)**2)
                if dist < 0.3:
                    return
            
            navigator.get_logger().info(f"New trash detected at map: ({tx:.3f}, {ty:.3f}, {tz:.3f})")
            navigator.detected_trash_queue.append((tx, ty, tz))
            navigator.new_trash_event.set()
                
    navigator.create_subscription(PoseStamped, '/detected_trash', trash_callback, 10)
    
    # Wait until simulation clock starts and progresses past 3.0 seconds
    # to ensure EKF filter and TF buffers are fully initialized.
    navigator.get_logger().info('Waiting for simulation clock to stabilize (> 3.0s)...')
    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)
        now = navigator.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()
        if sec >= 3:
            break
        time.sleep(0.5)
        
    # Initial pose setup if requested
    navigator.declare_parameter('set_initial_pose', False)
    if navigator.get_parameter('set_initial_pose').value:
        navigator.get_logger().info('Setting initial pose to (0,0,0)')
        navigator.setInitialPose(make_pose(navigator, 0.0, 0.0, 0.0))
        
    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active(localizer='amcl')
    navigator.get_logger().info('Nav2 active. Starting patrol and collect sequence.')
    
    try:
        while rclpy.ok():
            if navigator.state == 'PATROL':
                # Check if there is already something in the queue
                with navigator.lock:
                    if len(navigator.detected_trash_queue) > 0:
                        navigator.state = 'APPROACH'
                        continue
                
                # Perform normal patrol
                x, y = WAYPOINTS[navigator.current_wp_idx]
                nx, ny = WAYPOINTS[(navigator.current_wp_idx + 1) % len(WAYPOINTS)]
                yaw = math.atan2(ny - y, nx - x)
                
                label = f"WP{navigator.current_wp_idx + 1} ({x:+.1f},{y:+.1f})"
                navigator.get_logger().info(f"[State Transition] -> PATROL. Target: {label}")
                
                pose = make_pose(navigator, x, y, yaw)
                navigator.goToPose(pose)
                nav_start = navigator.get_clock().now()
                
                aborted = False
                while not navigator.isTaskComplete():
                    if navigator.new_trash_event.is_set():
                        navigator.get_logger().info("New trash event set! Aborting patrol task.")
                        navigator.new_trash_event.clear()
                        navigator.cancelTask()
                        aborted = True
                        with navigator.lock:
                            navigator.state = 'APPROACH'
                        break
                        
                    if (navigator.get_clock().now() - nav_start) > Duration(seconds=GOAL_TIMEOUT_SEC):
                        navigator.get_logger().warn(f"Patrol to {label} timed out. Cancelling.")
                        navigator.cancelTask()
                        aborted = True
                        break
                    
                    time.sleep(0.2)
                    
                if not aborted:
                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        navigator.get_logger().info(f"Reached {label} successfully.")
                        navigator.current_wp_idx = (navigator.current_wp_idx + 1) % len(WAYPOINTS)
                    else:
                        navigator.get_logger().warn(f"Failed to reach {label} (result={result}). Trying next.")
                        navigator.current_wp_idx = (navigator.current_wp_idx + 1) % len(WAYPOINTS)
                        
            elif navigator.state == 'APPROACH':
                with navigator.lock:
                    if not navigator.detected_trash_queue:
                        navigator.state = 'PATROL'
                        continue
                    navigator.current_target_trash = navigator.detected_trash_queue.pop(0)
                    
                tx, ty, tz = navigator.current_target_trash
                trash_id = len(navigator.processed_trash) + 1
                navigator.get_logger().info(f"[State Transition] -> APPROACH. Target Trash ID: {trash_id}, Pos: ({tx:.3f}, {ty:.3f})")
                
                success = False
                for attempt in [1, 2]:
                    navigator.get_logger().info(f"Approach attempt {attempt}/2...")
                    
                    rx, ry, ryaw = get_robot_pose(navigator)
                    if rx is None:
                        navigator.get_logger().warn("Could not get robot pose. Retrying.")
                        time.sleep(1.0)
                        continue
                        
                    theta = math.atan2(ty - ry, tx - rx)
                    goal_x = tx - APPROACH_DISTANCE * math.cos(theta)
                    goal_y = ty - APPROACH_DISTANCE * math.sin(theta)
                    
                    goal_pose = make_pose(navigator, goal_x, goal_y, theta)
                    navigator.get_logger().info(f"Going to approach pose: ({goal_x:.3f}, {goal_y:.3f}) facing {theta:.3f} rad")
                    
                    navigator.goToPose(goal_pose)
                    approach_start = navigator.get_clock().now()
                    aborted = False
                    
                    while not navigator.isTaskComplete():
                        if (navigator.get_clock().now() - approach_start) > Duration(seconds=90.0):
                            navigator.get_logger().warn("Approach timed out (90s limit). Cancelling.")
                            navigator.cancelTask()
                            aborted = True
                            break
                        time.sleep(0.2)
                        
                    result = navigator.getResult()
                    if not aborted and result == TaskResult.SUCCEEDED:
                        success = True
                        break
                    else:
                        navigator.get_logger().warn(f"Approach attempt {attempt} failed.")
                        
                if success:
                    rx, ry, ryaw = get_robot_pose(navigator)
                    dist = math.sqrt((tx - rx)**2 + (ty - ry)**2)
                    dx = tx - rx
                    dy = ty - ry
                    x_rel = dx * math.cos(ryaw) + dy * math.sin(ryaw)
                    y_rel = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
                    navigator.get_logger().info(f"Arrived at approach pose. Actual distance to trash: {dist:.3f}m, Relative: x_rel={x_rel:.3f}m, y_rel={y_rel:.3f}m")
                    with navigator.lock:
                        navigator.state = 'COLLECT'
                else:
                    navigator.get_logger().error(f"Approach to Trash ID: {trash_id} failed twice. Blacklisting.")
                    with navigator.lock:
                        navigator.blacklisted_trash.append(navigator.current_target_trash)
                        navigator.state = 'PATROL'
                        
            elif navigator.state == 'COLLECT':
                tx, ty, tz = navigator.current_target_trash
                trash_id = len(navigator.processed_trash) + 1
                navigator.get_logger().info(f"[State Transition] -> COLLECT. Target Trash ID: {trash_id}, Pos: ({tx:.3f}, {ty:.3f})")
                
                success = False
                
                # Check closest trash ID for logging
                closest_id = 1
                min_dist = float('inf')
                for i, (cx, cy) in TRASH_COORDINATES.items():
                    dist = math.sqrt((tx - cx)**2 + (ty - cy)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_id = i
                navigator.get_logger().info(f"Targeting Trash ID {closest_id} based on map coordinates.")

                for attempt in range(1, 3):
                    navigator.get_logger().info(f"Attempting manipulator pick-and-place (attempt {attempt}/2)...")
                    
                    # Compute relative coordinates to base_footprint for logging/debugging
                    rx, ry, ryaw = get_robot_pose(navigator)
                    if rx is not None:
                        dx = tx - rx
                        dy = ty - ry
                        x_rel = dx * math.cos(ryaw) + dy * math.sin(ryaw)
                        y_rel = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
                        navigator.get_logger().info(f"Relative position in base_footprint frame: x={x_rel:.3f}, y={y_rel:.3f}, z={tz:.3f}")
                    
                    if not navigator.pick_client.wait_for_service(timeout_sec=5.0):
                        navigator.get_logger().error("Pick-and-place service /pick_trash not available!")
                        time.sleep(2.0)
                        continue
                    
                    req = GetPlan.Request()
                    req.goal = PoseStamped()
                    req.goal.header.frame_id = 'map'
                    req.goal.header.stamp = navigator.get_clock().now().to_msg()
                    req.goal.pose.position.x = float(tx)
                    req.goal.pose.position.y = float(ty)
                    req.goal.pose.position.z = float(tz)
                    req.goal.pose.orientation.w = 1.0
                    
                    future = navigator.pick_client.call_async(req)
                    
                    # Spin until future completes with a 120s timeout
                    start_time = time.monotonic()
                    while not future.done() and (time.monotonic() - start_time < 120.0) and rclpy.ok():
                        time.sleep(0.1)
                        
                    if future.done():
                        res = future.result()
                        if res is not None and len(res.plan.poses) > 0:
                            navigator.get_logger().info(f"回収完了: id={trash_id}")
                            success = True
                            break
                        else:
                            navigator.get_logger().warn(f"Manipulator pick-and-place attempt {attempt} failed.")
                    else:
                        navigator.get_logger().warn(f"Manipulator pick-and-place attempt {attempt} timed out.")
                        
                    # If first attempt failed, adjust vehicle position (0.05m forward)
                    if attempt == 1:
                        navigator.get_logger().info("First pick attempt failed. Attempting recovery: adjusting vehicle position 0.05m forward...")
                        rx, ry, ryaw = get_robot_pose(navigator)
                        if rx is not None:
                            adj_x = rx + 0.05 * math.cos(ryaw)
                            adj_y = ry + 0.05 * math.sin(ryaw)
                            adj_pose = make_pose(navigator, adj_x, adj_y, ryaw)
                            navigator.get_logger().info(f"Moving to adjusted pose: ({adj_x:.3f}, {adj_y:.3f})")
                            navigator.goToPose(adj_pose)
                            
                            move_start = navigator.get_clock().now()
                            aborted = False
                            while not navigator.isTaskComplete():
                                if (navigator.get_clock().now() - move_start) > Duration(seconds=30.0):
                                    navigator.get_logger().warn("Adjustment movement timed out (30s limit). Cancelling.")
                                    navigator.cancelTask()
                                    aborted = True
                                    break
                                time.sleep(0.1)
                            
                            result = navigator.getResult()
                            if not aborted and result == TaskResult.SUCCEEDED:
                                navigator.get_logger().info("Successfully moved 0.05m forward for adjustment.")
                            else:
                                navigator.get_logger().warn("Failed to move forward for adjustment.")
                            time.sleep(1.0) # Wait a bit before retry
                
                with navigator.lock:
                    if success:
                        navigator.processed_trash.append(navigator.current_target_trash)
                    else:
                        navigator.get_logger().error(f"Manipulator pick-and-place failed after 2 attempts. Blacklisting Trash ID: {trash_id}")
                        navigator.blacklisted_trash.append(navigator.current_target_trash)
                    navigator.state = 'PATROL'
                    
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        navigator.get_logger().info('Ctrl+C: Cancelling current task and exiting.')
        navigator.cancelTask()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
