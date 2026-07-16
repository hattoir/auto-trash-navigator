#!/usr/bin/env python3
"""Task 3: ピック＆プレースシーケンスノード
"""
import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from nav_msgs.srv import GetPlan
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs

# Trash models and initial locations to identify the closest simulation trash ID
TRASH_COORDINATES = {
    1: (2.2, -1.8),
    2: (0.2, 2.2),
    3: (-2.2, 0.2),
}

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_server', parameter_overrides=[
            rclpy.Parameter('use_sim_time', value=True)
        ])
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        self.current_joint_state = None
        
        # Service Servers
        # Using nav_msgs/srv/GetPlan to accept goal PoseStamped in the request
        self.pick_service = self.create_service(
            GetPlan,
            '/pick_trash',
            self.pick_trash_callback,
            callback_group=self.callback_group
        )
        
        # Service Clients
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )
        
        # Action Clients
        self.arm_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        self.gripper_action = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Publishers
        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10,
            callback_group=self.callback_group
        )
        
        # DetachableJoint attach/detach publishers
        self.attach_pubs = {}
        self.detach_pubs = {}
        for i in range(1, 4):
            self.attach_pubs[i] = self.create_publisher(Empty, f'/attach_trash_{i}', 10, callback_group=self.callback_group)
            self.detach_pubs[i] = self.create_publisher(Empty, f'/detach_trash_{i}', 10, callback_group=self.callback_group)
            
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Pick and Place Server initialized.")
        
        # Default patrol joints fallback
        self.patrol_joints = [0.0, 0.0, 0.2, 0.0, 1.2, 0.0]
        
        # Spawn thread to move arm to patrol posture after node starts
        threading.Thread(target=self.initialize_patrol_pose, daemon=True).start()
        
    def initialize_patrol_pose(self):
        # Wait for simulation and controller services to settle
        time.sleep(5.0)
        self.get_logger().info("Using UPRIGHT patrol posture to keep camera FOV clear...")
        self.patrol_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
        self.get_logger().info("Moving arm to default patrol camera look-down pose...")
        self.send_arm_trajectory(self.patrol_joints, 3.0)
        self.get_logger().info("Arm is in patrol camera look-down pose.")
        
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        
    def publish_planning_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        
        # 1. Floor Collision Object
        floor = CollisionObject()
        floor.header.frame_id = 'base_footprint'
        floor.id = 'floor'
        
        primitive_floor = SolidPrimitive()
        primitive_floor.type = SolidPrimitive.BOX
        primitive_floor.dimensions = [10.0, 10.0, 0.01]
        
        pose_floor = Pose()
        pose_floor.position.x = 0.0
        pose_floor.position.y = 0.0
        pose_floor.position.z = -0.005
        pose_floor.orientation.w = 1.0
        
        floor.primitives.append(primitive_floor)
        floor.primitive_poses.append(pose_floor)
        floor.operation = CollisionObject.ADD
        
        scene.world.collision_objects.append(floor)
        
        # 2. Vehicle Body Collision Object
        body = CollisionObject()
        body.header.frame_id = 'base_footprint'
        body.id = 'vehicle_body'
        
        primitive_body = SolidPrimitive()
        primitive_body.type = SolidPrimitive.BOX
        primitive_body.dimensions = [0.4, 0.4, 0.15]
        
        pose_body = Pose()
        pose_body.position.x = 0.0
        pose_body.position.y = 0.0
        pose_body.position.z = 0.10  # base_link is at z=0.1 from base_footprint
        pose_body.orientation.w = 1.0
        
        body.primitives.append(primitive_body)
        body.primitive_poses.append(pose_body)
        body.operation = CollisionObject.ADD
        
        scene.world.collision_objects.append(body)
        
        self.scene_pub.publish(scene)
        self.get_logger().info("Published planning scene objects (floor and vehicle body).")
        
    def send_arm_trajectory(self, positions, duration_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points.append(point)
        
        self.arm_action.wait_for_server()
        future = self.arm_action.send_goal_async(goal)
        
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 5.0):
            time.sleep(0.05)
            
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                start_time = time.monotonic()
                while not result_future.done() and (time.monotonic() - start_time < duration_sec + 2.0):
                    time.sleep(0.05)
                return True
        return False
        
    def send_gripper_trajectory(self, position, duration_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger_joint', 'right_finger_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(position), float(position)]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points.append(point)
        
        self.gripper_action.wait_for_server()
        future = self.gripper_action.send_goal_async(goal)
        
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 5.0):
            time.sleep(0.05)
            
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                start_time = time.monotonic()
                while not result_future.done() and (time.monotonic() - start_time < duration_sec + 2.0):
                    time.sleep(0.05)
                return True
        return False

    def solve_ik(self, x, y, z, pitch, yaw):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service /compute_ik not available.")
            return None
            
        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = 'arm'
        ik_req.ik_link_name = 'link6'
        ik_req.avoid_collisions = False
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_footprint'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(x)
        pose_stamped.pose.position.y = float(y)
        pose_stamped.pose.position.z = float(z)
        
        # Angled down gripper orientation (from reach_map.py Condition 2)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        
        pose_stamped.pose.orientation.x = -sp * sy
        pose_stamped.pose.orientation.y = sp * cy
        pose_stamped.pose.orientation.z = cp * sy
        pose_stamped.pose.orientation.w = cp * cy
        
        ik_req.pose_stamped = pose_stamped
        
        # Set seed state to look_down pose to guide KDL solver to forward-reaching solutions
        robot_state = RobotState()
        robot_state.joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        robot_state.joint_state.position = [float(yaw), 1.2, 1.5, 0.0, -0.8, 0.0]
        ik_req.robot_state = robot_state
            
        req.ik_request = ik_req
        
        future = self.ik_client.call_async(req)
        # Wait for the service response asynchronously using time.sleep.
        # The MultiThreadedExecutor is spinning on another thread.
        start_time = time.monotonic()
        while not future.done() and (time.monotonic() - start_time < 2.0):
            time.sleep(0.02)
            
        if future.done():
            res = future.result()
            if res.error_code.val == 1: # SUCCESS
                # Filter out gripper joints
                arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                positions = []
                for name in arm_joints:
                    idx = res.solution.joint_state.name.index(name)
                    positions.append(res.solution.joint_state.position[idx])
                return positions
            else:
                self.get_logger().warn(f"IK failed with error code: {res.error_code.val}")
                return None
        return None

    def pick_trash_callback(self, request, response):
        self.get_logger().info("Received pick trash request!")
        
        # Update Planning Scene
        self.publish_planning_scene()
        
        # 1. Transform goal pose from request (usually 'map') to 'base_footprint'
        goal_pose_map = request.goal
        goal_pose_base = None
        if goal_pose_map.header.frame_id in ['base_footprint', '']:
            goal_pose_base = goal_pose_map.pose
        else:
            try:
                # We want to transform to 'base_footprint'
                transform = self.tf_buffer.lookup_transform(
                    'base_footprint',
                    goal_pose_map.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=2.0)
                )
                goal_pose_base = tf2_geometry_msgs.do_transform_pose(goal_pose_map.pose, transform)
            except Exception as e:
                self.get_logger().warn(f"Failed to transform pose ({e}). Falling back to raw coordinates as base_footprint.")
                goal_pose_base = goal_pose_map.pose
                
        if goal_pose_base is not None:
            self.get_logger().info(f"Using trash pose for base_footprint: "
                                    f"x={goal_pose_base.position.x:.3f}, "
                                    f"y={goal_pose_base.position.y:.3f}, "
                                    f"z={goal_pose_base.position.z:.3f}")
            
        # Target coordinate in base_footprint
        tx = goal_pose_base.position.x
        ty = goal_pose_base.position.y
        
        # 2. Identify the closest trash in the simulation to trigger DetachableJoint
        # Transform goal back or compare in map frame to TRASH_COORDINATES
        mx = goal_pose_map.pose.position.x
        my = goal_pose_map.pose.position.y
        closest_id = 1
        min_dist = float('inf')
        for i, (cx, cy) in TRASH_COORDINATES.items():
            dist = math.sqrt((mx - cx)**2 + (my - cy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_id = i
                
        self.get_logger().info(f"Targeting Trash ID: {closest_id} (distance in map: {min_dist:.3f}m)")
        
        # --- Start Picking Sequence ---
        # Pitch: 135 degrees (angled down, pointing at ground), Yaw: facing the target
        target_yaw = math.atan2(ty, tx)
        target_pitch = 135.0 * math.pi / 180.0
        
        # Abort helper to handle step failures
        def abort_sequence(error_msg):
            self.get_logger().error(f"Abort triggered: {error_msg}")
            # Safely release any attached object
            self.detach_pubs[closest_id].publish(Empty())
            # Open gripper (0.0)
            self.send_gripper_trajectory(0.0, 1.0)
            # Move to patrol pose
            self.send_arm_trajectory(self.patrol_joints, 2.5)
            self.get_logger().info("Aborted sequence. Arm returned to look-down patrol pose.")
        
        # 1. home (patrol pose)
        self.get_logger().info("[Step 1/11] Moving arm to home look-down patrol pose...")
        if not self.send_arm_trajectory(self.patrol_joints, 2.5):
            abort_sequence("Failed to move to Home patrol pose at start.")
            return response
            
        # 2. プリグラスプ (対象の上方8cm, pitch45deg)
        self.get_logger().info("[Step 2/11] Moving to Pre-grasp pose (z = target_z + 0.08)...")
        pre_grasp_z = goal_pose_base.position.z + 0.08
        pre_grasp_joints = self.solve_ik(tx, ty, pre_grasp_z, target_pitch, target_yaw)
        if pre_grasp_joints is None:
            self.get_logger().warn("IK failed at 8cm above. Trying 4.5cm above (z = target_z + 0.045)...")
            pre_grasp_z = goal_pose_base.position.z + 0.045
            pre_grasp_joints = self.solve_ik(tx, ty, pre_grasp_z, target_pitch, target_yaw)
        if pre_grasp_joints is None:
            abort_sequence(f"IK failed for Pre-grasp pose at both 8cm and 4.5cm (x={tx:.3f}, y={ty:.3f})")
            return response
        if not self.send_arm_trajectory(pre_grasp_joints, 2.5):
            abort_sequence("Failed to execute Pre-grasp joint trajectory.")
            return response
            
        # 3. 開爪
        self.get_logger().info("[Step 3/11] Opening gripper...")
        if not self.send_gripper_trajectory(0.0, 1.0):
            abort_sequence("Failed to open gripper.")
            return response
            
        # 4. 接近 (対象位置)
        self.get_logger().info("[Step 4/11] Moving down to Grasp pose (z = target_z)...")
        grasp_z = goal_pose_base.position.z
        grasp_joints = self.solve_ik(tx, ty, grasp_z, target_pitch, target_yaw)
        if grasp_joints is None:
            abort_sequence(f"IK failed for Grasp pose (x={tx:.3f}, y={ty:.3f}, z={grasp_z:.3f})")
            return response
        if not self.send_arm_trajectory(grasp_joints, 2.0):
            abort_sequence("Failed to execute Grasp joint trajectory.")
            return response
            
        # 5. 吸着 (detachable joint attach)
        self.get_logger().info("[Step 5/11] Attaching trash via DetachableJoint...")
        self.attach_pubs[closest_id].publish(Empty())
        time.sleep(0.5) # Allow simulation to process attachment
        
        # 6. 閉爪
        self.get_logger().info("[Step 6/11] Closing gripper...")
        if not self.send_gripper_trajectory(0.015, 1.0):
            abort_sequence("Failed to close gripper.")
            return response
            
        # 7. 持ち上げ (上方10cm)
        self.get_logger().info("[Step 7/11] Lifting up trash (z = target_z + 0.10)...")
        lift_z = goal_pose_base.position.z + 0.10
        lift_joints = self.solve_ik(tx, ty, lift_z, target_pitch, target_yaw)
        if lift_joints is None:
            self.get_logger().warn("IK failed for 10cm lift. Trying 5cm lift (z = target_z + 0.05)...")
            lift_z = goal_pose_base.position.z + 0.05
            lift_joints = self.solve_ik(tx, ty, lift_z, target_pitch, target_yaw)
        if lift_joints is None:
            # Try fallback raise
            self.get_logger().warn("IK failed for Lift pose. Attempting fallback direct joint lift.")
            if len(grasp_joints) >= 3:
                lift_joints = list(grasp_joints)
                lift_joints[1] -= 0.2
                lift_joints[2] += 0.2
        if lift_joints is None:
            abort_sequence("IK failed and no fallback joints available for Lift pose.")
            return response
        if not self.send_arm_trajectory(lift_joints, 2.0):
            abort_sequence("Failed to execute Lift joint trajectory.")
            return response
            
        # 8. drop_pose
        self.get_logger().info("[Step 8/11] Moving to Drop pose...")
        # SRDF drop_pose: joint1=1.57, joint2=-0.2, joint3=0.4, joint4=0, joint5=0.4, joint6=0
        drop_joints = [1.57, -0.2, 0.4, 0.0, 0.4, 0.0]
        if not self.send_arm_trajectory(drop_joints, 2.5):
            abort_sequence("Failed to move to Drop pose.")
            return response
            
        # 9. 解放 (detach)
        self.get_logger().info("[Step 9/11] Detaching trash via DetachableJoint...")
        self.detach_pubs[closest_id].publish(Empty())
        time.sleep(0.5) # Allow simulation to process detachment
        
        # 10. 開爪
        self.get_logger().info("[Step 10/11] Opening gripper...")
        if not self.send_gripper_trajectory(0.0, 1.0):
            abort_sequence("Failed to open gripper at drop.")
            return response
            
        # 11. home (patrol pose)
        self.get_logger().info("[Step 11/11] Returning to Home look-down patrol pose...")
        if not self.send_arm_trajectory(self.patrol_joints, 2.5):
            self.get_logger().error("Failed to return to Home patrol pose at end.")
            return response
            
        self.get_logger().info("Pick and place sequence successfully completed!")
        
        # Return a dummy path in the response to indicate success
        dummy_pose = PoseStamped()
        dummy_pose.header.stamp = self.get_clock().now().to_msg()
        response.plan.poses.append(dummy_pose)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
