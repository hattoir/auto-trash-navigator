#!/usr/bin/env python3
"""Task 3: ピック＆プレースシーケンスノード (Pose追従方式 + 実座標検出)

注意: 本実装は「回収」を gz service set_pose によるテレポートで実現する
シミュレーション専用方式であり、実機の物理的な把持とは非互換。
実機移行時は把持機構(グリッパ/吸着)ベースの実装に置き換えること。
"""
import math
import time
import threading
import subprocess
import re
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from sensor_msgs.msg import JointState
from nav_msgs.srv import GetPlan
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs

# Initial coordinates fallback
TRASH_COORDINATES = {
    1: (2.2, -1.8),
    2: (0.4, 2.0),
    3: (-2.0, -1.2),
}

# Fixed drop destination (world "dustbox" model, office_room.sdf).
# sim専用: 実機ではアーム投下そのもので実現。
DUSTBOX_LOCATION = (3.5, -3.5, 0.35)

# グリッパー可動域 (実機CAD実測、gripper.xacro参照): -0.013〜+0.013 m
# (0 = 中央34mm開口)。開口[mm] = 34 + 1846 * joint[m]。
# GRIPPER_OPEN: 全開(開口60mm)。GRIPPER_CLOSED_GRASP: 紙くず(直径約24mm)を
# 掴むための閉じ量(開口 34 - 1846*0.004 ≈ 26.6mm、対象の24mmより少し広く
# 余裕を持たせた値)。
GRIPPER_OPEN = 0.013
GRIPPER_CLOSED_GRASP = -0.004

# 把持姿勢のピッチ角(可達範囲実測に基づく): 90°(真下)未満の傾斜姿勢は
# 60°が上限(45°はJ2可動域±1.65radを超えるため物理的に到達不可)。
# ここでの角度はeuler_to_quaternion(0, pitch, yaw)への入力値で、
# タスクの「pitch」表記(水平からの角度)+90°に相当する。
TARGET_PITCH_RAD = 150.0 * math.pi / 180.0

# --- 混合姿勢方式(Pre-grasp/Lift は垂直寄り、Grasp瞬間のみ60°) ---
# 実測(compute_ik走査)で、Grasp姿勢(pitch=60°)は到達域が床付近
# (z≈0.02)のごく薄い層に限られ、8cm/4.5cm上方でホバーする従来設計とは
# 両立しないことが判明した。一方 pitch=90°(真下)は同じ(x,y)で
# z=0.02〜0.20の広い範囲に到達できるため、Pre-grasp/LiftはPitch=90°で
# 実施し、Grasp直前だけ pitch=60° へ遷移する。
# PRE_GRASP_Zは固定値ではなく、対象座標ごとに以下の候補を高い方から
# 順にIKで試し、最初に解けた高さを採用する(実行時決定)。
# 単体検証で、接近目標(0.32,0.10)ちょうどではz=0.10が解けても、
# 検出誤差(±0.03m程度)がある座標では z=0.10 が解けないことが判明した
# (例: (0.29,0.07)はz=0.08以下でのみ解け、(0.35,0.13)はz=0.12以上
# でのみ解ける、という逆方向の傾向がある)。高い方から低い方へ順に
# 試すラダー探索であれば、どちらの傾向の座標にも対応できる。
PRE_GRASP_Z_CANDIDATES = [0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.04, 0.02]
PRE_GRASP_PITCH_RAD = math.pi  # タスクのpitch=90°(真下)に相当
DESCENT_STEPS = 10  # Pre-grasp -> Grasp を10分割し、pitch/zを線形補間

# --- 把持目標探索(2026-08-09 自律改善ループ2) ---
# 統合検証で、Nav2の実到着誤差(y方向5〜6cm)がGrasp到達域の許容(±3cm)を
# 上回り、車体の停止精度だけでは埋まらないことが判明した。そこで
# 「車体で正確に合わせる」のをやめ、「到着後にアーム側で解ける点を探す」
# 設計に変更する。対象(紙くず、直径約24mm)は検出座標を中心に
# ±0.05mずれてもグリッパー開口(最大60mm)内に収まるため、検出座標
# そのものではなく、その近傍でPre-grasp/降下経路/Graspの全区間が
# IKで解ける点を能動的に探索し、そこを把持目標として採用する。
# 探索順序は対象中心に近い候補から試す(グリップ精度を優先)。
GRASP_SEARCH_STEP = 0.02
GRASP_SEARCH_MAX = 0.05
_offsets_1d = sorted(
    {round(v, 3) for v in
     [0.0] +
     [i * GRASP_SEARCH_STEP for i in range(1, int(GRASP_SEARCH_MAX / GRASP_SEARCH_STEP) + 1)] +
     [-i * GRASP_SEARCH_STEP for i in range(1, int(GRASP_SEARCH_MAX / GRASP_SEARCH_STEP) + 1)] +
     [GRASP_SEARCH_MAX, -GRASP_SEARCH_MAX]},
    key=abs
)
GRASP_SEARCH_OFFSETS = sorted(
    [(dx, dy) for dx in _offsets_1d for dy in _offsets_1d],
    key=lambda p: p[0] ** 2 + p[1] ** 2
)

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

def get_trash_real_pose(trash_id):
    try:
        res = subprocess.run(["gz", "model", "-m", f"paper_trash_{trash_id}", "-p"], capture_output=True, text=True, timeout=2.0)
        lines = res.stdout.splitlines()
        for idx, line in enumerate(lines):
            if "Pose [ XYZ" in line and idx + 1 < len(lines):
                pos_line = lines[idx + 1].strip()
                match = re.search(r'\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]', pos_line)
                if match:
                    return float(match.group(1)), float(match.group(2))
    except Exception:
        pass
    return TRASH_COORDINATES.get(trash_id, (0.0, 0.0))

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
        
        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Pose tracking state
        self.tracking_running = False
        self.tracking_thread = None
        self.tracking_trash_name = None
        self.tracking_lock = threading.Lock()
        
        self.get_logger().info("Pick and Place Server initialized.")
        
        # Default patrol joints fallback
        self.patrol_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Spawn thread to move arm to patrol posture after node starts
        threading.Thread(target=self.initialize_patrol_pose, daemon=True).start()

    def start_pose_tracking(self, trash_name):
        with self.tracking_lock:
            self.stop_pose_tracking_unlocked()
            self.tracking_trash_name = trash_name
            self.tracking_running = True
            self.tracking_thread = threading.Thread(target=self._pose_tracking_loop, daemon=True)
            self.tracking_thread.start()
            self.get_logger().info(f"Started pose tracking for {trash_name}")

    def stop_pose_tracking(self):
        with self.tracking_lock:
            return self.stop_pose_tracking_unlocked()

    def stop_pose_tracking_unlocked(self):
        last_pos = None
        if self.tracking_running:
            self.tracking_running = False
            if self.tracking_thread and self.tracking_thread.is_alive():
                self.tracking_thread.join(timeout=1.0)
            self.tracking_thread = None
            
            try:
                trans = self.tf_buffer.lookup_transform('map', 'link6', rclpy.time.Time())
                last_pos = (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
            except Exception:
                pass
            self.get_logger().info(f"Stopped pose tracking for {self.tracking_trash_name}")
            self.tracking_trash_name = None
        return last_pos

    def _pose_tracking_loop(self):
        rate_sec = 0.1 # 10Hz
        while self.tracking_running:
            try:
                trans = self.tf_buffer.lookup_transform('map', 'link6', rclpy.time.Time())
                tx = trans.transform.translation.x
                ty = trans.transform.translation.y
                tz = trans.transform.translation.z + 0.03
                
                req_str = f'name: "{self.tracking_trash_name}", position: {{x: {tx:.4f}, y: {ty:.4f}, z: {tz:.4f}}}'
                cmd = [
                    'gz', 'service', '-s', '/world/office_room/set_pose',
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '500',
                    '--req', req_str
                ]
                subprocess.run(cmd, capture_output=True)
            except Exception:
                pass
            time.sleep(rate_sec)
        
    def initialize_patrol_pose(self):
        # Wait 15.0 seconds for move_group to start up and register joint trajectory controllers
        time.sleep(15.0)
        self.get_logger().info("Using UPRIGHT patrol posture to keep camera FOV clear...")
        self.patrol_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Moving arm to default patrol camera look-down pose...")
        if self.send_arm_trajectory(self.patrol_joints, 3.0):
            self.get_logger().info("Arm is in patrol camera look-down pose.")
        else:
            self.get_logger().warn("Arm trajectory execution failed at startup. Controller might not be ready yet.")

        
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        
    def publish_planning_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        
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
        self.scene_pub.publish(scene)
        
    def send_arm_trajectory(self, joint_positions, duration=3.0):
        if not self.arm_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm action server not available!")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(point)
        
        send_goal_future = self.arm_action.send_goal_async(goal_msg)
        
        start_t = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_t > 10.0:
                self.get_logger().error("Timeout waiting for arm goal response!")
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Arm trajectory goal rejected!")
            return False
            
        get_result_future = goal_handle.get_result_async()
        
        start_t = time.time()
        while not get_result_future.done():
            time.sleep(0.05)
            if time.time() - start_t > (duration + 5.0):
                self.get_logger().error("Timeout waiting for arm trajectory execution!")
                return False
        
        res = get_result_future.result()
        return res is not None
        
    def send_gripper_trajectory(self, position, duration=1.0):
        if not self.gripper_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available!")
            return False
            
        # right_finger_joint mimics left_finger_joint (gripper.xacro) and is
        # driven by the physics engine's mimic constraint; only the primary
        # joint is commanded here.
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['left_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal_msg.trajectory.points.append(point)
        
        send_goal_future = self.gripper_action.send_goal_async(goal_msg)
        
        start_t = time.time()
        while not send_goal_future.done():
            time.sleep(0.05)
            if time.time() - start_t > 5.0:
                self.get_logger().error("Timeout waiting for gripper goal response!")
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Gripper trajectory goal rejected!")
            return False
            
        get_result_future = goal_handle.get_result_async()
        
        start_t = time.time()
        while not get_result_future.done():
            time.sleep(0.05)
            if time.time() - start_t > (duration + 3.0):
                self.get_logger().error("Timeout waiting for gripper trajectory execution!")
                return False
        
        res = get_result_future.result()
        return res is not None

    def solve_ik(self, target_x, target_y, target_z, target_pitch, target_yaw):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik service not available!")
            return None
            
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.avoid_collisions = False
        req.ik_request.ik_link_name = 'grasp_link'
        
        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state
            
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_footprint'
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(target_x)
        target_pose.pose.position.y = float(target_y)
        target_pose.pose.position.z = float(target_z)
        
        qx, qy, qz, qw = euler_to_quaternion(0.0, target_pitch, target_yaw)
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw
        
        req.ik_request.pose_stamped = target_pose
        
        future = self.ik_client.call_async(req)
        
        start_t = time.time()
        while not future.done():
            time.sleep(0.05)
            if time.time() - start_t > 5.0:
                self.get_logger().error("Timeout waiting for /compute_ik response!")
                return None
        
        res = future.result()
        if res and res.error_code.val == res.error_code.SUCCESS:
            arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            positions = []
            name_pos_map = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
            for name in arm_joint_names:
                if name in name_pos_map:
                    positions.append(name_pos_map[name])
                else:
                    self.get_logger().error(f"Joint {name} missing in IK solution!")
                    return None
            return positions
        else:
            err_code = res.error_code.val if res else 'None'
            self.get_logger().warn(f"IK failed with error code: {err_code}")
            return None

    def find_grasp_target(self, tx, ty, tz_val, target_yaw):
        """検出座標(tx,ty)近傍の候補点(GRASP_SEARCH_OFFSETS)を近い順に走査し、
        Pre-grasp高さ探索+降下経路(10分割)+Graspまで全区間がIKで解ける
        最初の点を返す。見つからなければNoneを返す。"""
        for dx, dy in GRASP_SEARCH_OFFSETS:
            cand_tx = tx + dx
            cand_ty = ty + dy

            pre_grasp_joints = None
            pre_grasp_z = None
            for cand_z in PRE_GRASP_Z_CANDIDATES:
                pre_grasp_joints = self.solve_ik(cand_tx, cand_ty, cand_z, PRE_GRASP_PITCH_RAD, target_yaw)
                if pre_grasp_joints is not None:
                    pre_grasp_z = cand_z
                    break
            if pre_grasp_joints is None:
                continue

            descent_waypoints = []
            path_ok = True
            for i in range(1, DESCENT_STEPS + 1):
                frac = i / DESCENT_STEPS
                wp_z = pre_grasp_z + (tz_val - pre_grasp_z) * frac
                wp_pitch = PRE_GRASP_PITCH_RAD + (TARGET_PITCH_RAD - PRE_GRASP_PITCH_RAD) * frac
                wp_joints = self.solve_ik(cand_tx, cand_ty, wp_z, wp_pitch, target_yaw)
                if wp_joints is None:
                    path_ok = False
                    break
                descent_waypoints.append(wp_joints)
            if not path_ok:
                continue

            self.get_logger().info(
                f"[Grasp Search] Found solvable point at offset (dx={dx:+.2f}, dy={dy:+.2f}) "
                f"from detected coords -> target=({cand_tx:.3f},{cand_ty:.3f}), pre_grasp_z={pre_grasp_z}")
            return {
                'tx': cand_tx, 'ty': cand_ty,
                'dx': dx, 'dy': dy,
                'pre_grasp_joints': pre_grasp_joints, 'pre_grasp_z': pre_grasp_z,
                'descent_waypoints': descent_waypoints,
            }
        return None

    def pick_trash_callback(self, request, response):
        self.get_logger().info("Received /pick_trash request.")
        self.publish_planning_scene()
        
        goal_pose_input = request.goal
        goal_pose_base = None
        goal_pose_map = None
        
        if goal_pose_input.header.frame_id == 'base_footprint':
            goal_pose_base = goal_pose_input
            try:
                goal_pose_map = self.tf_buffer.transform(goal_pose_input, 'map', timeout=rclpy.duration.Duration(seconds=2.0))
            except Exception as e:
                self.get_logger().warn(f"Transform to map failed: {e}")
                goal_pose_map = goal_pose_input
        else:
            goal_pose_map = goal_pose_input
            try:
                goal_pose_base = self.tf_buffer.transform(goal_pose_input, 'base_footprint', timeout=rclpy.duration.Duration(seconds=2.0))
            except Exception as e:
                self.get_logger().warn(f"Transform to base_footprint failed: {e}")
                goal_pose_base = goal_pose_input
                
        if goal_pose_base is not None and hasattr(goal_pose_base, 'pose'):
            tx = goal_pose_base.pose.position.x
            ty = goal_pose_base.pose.position.y
            tz_val = goal_pose_base.pose.position.z
        else:
            tx = goal_pose_input.pose.position.x
            ty = goal_pose_input.pose.position.y
            tz_val = goal_pose_input.pose.position.z
            
        mx = goal_pose_map.pose.position.x if hasattr(goal_pose_map, 'pose') else tx
        my = goal_pose_map.pose.position.y if hasattr(goal_pose_map, 'pose') else ty

        self.get_logger().info(f"Target coords: base=({tx:.3f}, {ty:.3f}), map=({mx:.3f}, {my:.3f})")

        # Identify closest trash using real model positions
        closest_id = None
        min_dist = float('inf')
        for i in range(1, 4):
            cx, cy = get_trash_real_pose(i)
            dist = math.sqrt((mx - cx)**2 + (my - cy)**2)
            if dist < min_dist:
                min_dist = dist
                closest_id = i
                
        if closest_id is None or min_dist > 0.6:
            self.get_logger().error(f"Refusing pick: No trash within 0.6m of target coordinate ({mx:.3f}, {my:.3f}). Min dist: {min_dist:.3f}m")
            return response
            
        trash_name = f"paper_trash_{closest_id}"
        self.get_logger().info(f"Targeting Trash ID: {closest_id} ({trash_name}, distance in map: {min_dist:.3f}m)")
        
        target_yaw = math.atan2(ty, tx)

        def abort_sequence(error_msg):
            self.get_logger().error(f"Abort triggered: {error_msg}")
            self.stop_pose_tracking()
            self.send_gripper_trajectory(GRIPPER_OPEN, 1.0)
            self.send_arm_trajectory(self.patrol_joints, 2.5)
            self.get_logger().info("Aborted sequence. Arm returned to look-down patrol pose.")
        
        # A. home (patrol pose)
        self.get_logger().info("[Step A] Moving arm to home look-down patrol pose...")
        if not self.send_arm_trajectory(self.patrol_joints, 2.5):
            abort_sequence("Failed to move to Home patrol pose at start.")
            return response

        # B+C. 把持目標探索: 検出座標(tx,ty)近傍(±0.05m, 0.02m刻み)で
        # Pre-grasp高さ探索+降下経路(10分割)+Graspの全区間がIKで解ける
        # 点を、対象に近い順に探す。対象(直径約24mm)はグリッパー開口
        # (最大60mm)に対して十分な余裕があるため、検出座標そのものに
        # 拘らず、解ける近傍点があればそこを把持目標として採用する。
        self.get_logger().info(f"[Step B] Searching grasp target near ({tx:.3f},{ty:.3f}) "
                                f"(offsets up to +-{GRASP_SEARCH_MAX}m, step {GRASP_SEARCH_STEP}m)...")
        grasp_target = self.find_grasp_target(tx, ty, tz_val, target_yaw)
        if grasp_target is None:
            abort_sequence(f"No solvable grasp target found within +-{GRASP_SEARCH_MAX}m of "
                            f"(x={tx:.3f}, y={ty:.3f})")
            return response
        gtx, gty = grasp_target['tx'], grasp_target['ty']
        pre_grasp_joints = grasp_target['pre_grasp_joints']
        pre_grasp_z = grasp_target['pre_grasp_z']
        descent_waypoints = grasp_target['descent_waypoints']
        self.get_logger().info(f"[Step B] Pre-grasp height found: z={pre_grasp_z} "
                                f"(grasp target offset dx={grasp_target['dx']:+.2f}, dy={grasp_target['dy']:+.2f})")

        if not self.send_arm_trajectory(pre_grasp_joints, 2.5):
            abort_sequence("Failed to execute Pre-grasp joint trajectory.")
            return response

        self.get_logger().info("[Step B] Opening gripper...")
        if not self.send_gripper_trajectory(GRIPPER_OPEN, 1.0):
            abort_sequence("Failed to open gripper at Pre-grasp.")
            return response

        # C. 降下経路の実行(探索済みの10分割ウェイポイントをそのまま使用)
        self.get_logger().info(f"[Step C] Executing descent path ({DESCENT_STEPS} waypoints, "
                                f"already IK-verified during grasp target search)...")
        for i, wp_joints in enumerate(descent_waypoints, start=1):
            if not self.send_arm_trajectory(wp_joints, 0.4):
                abort_sequence(f"Failed to execute descent waypoint {i}/{DESCENT_STEPS}.")
                return response
        grasp_joints = descent_waypoints[-1]

        # D. 吸着 (Start pose tracking thread) + 閉爪
        self.get_logger().info(f"[Step D] Attaching trash {trash_name} via Pose Tracking...")
        self.start_pose_tracking(trash_name)
        time.sleep(0.3)

        self.get_logger().info("[Step D] Closing gripper...")
        if not self.send_gripper_trajectory(GRIPPER_CLOSED_GRASP, 1.0):
            abort_sequence("Failed to close gripper.")
            return response

        # E. Lift (Step Bで見つかった同じ高さ・pitch=90°へ戻る。
        # 到達確認済みの座標のため、再探索は不要)
        self.get_logger().info(f"[Step E] Lifting to Pre-grasp height (z={pre_grasp_z}, pitch=90deg)...")
        lift_joints = self.solve_ik(gtx, gty, pre_grasp_z, PRE_GRASP_PITCH_RAD, target_yaw)
        if lift_joints is None:
            abort_sequence(f"IK failed for Lift pose (x={gtx:.3f}, y={gty:.3f}, z={pre_grasp_z:.3f})")
            return response
        if not self.send_arm_trajectory(lift_joints, 2.0):
            abort_sequence("Failed to execute Lift joint trajectory.")
            return response

        # F. drop_pose (SRDF drop_pose group_state: grasp_link=(0.10,0.25,0.25)
        # pitch=60deg, IK/FK検証済み 2026-08-07)
        self.get_logger().info("[Step F] Moving to Drop pose...")
        drop_joints = [1.9652, 0.6362, 0.862, -0.3746, 1.2729, -0.7525]
        if not self.send_arm_trajectory(drop_joints, 2.5):
            abort_sequence("Failed to move to Drop pose.")
            return response

        # 解放 (Stop pose tracking and final teleport to dustbox)
        # sim専用: 実機ではアーム投下そのもので実現(ドロップ姿勢へ移動して
        # 開爪するだけでダストボックスに入る設計とする)。ここでは
        # アーム動作(drop_joints)は演出として維持しつつ、最終座標だけを
        # ワールド固定のダストボックス位置に差し替える。
        self.get_logger().info(f"[Step F] Detaching trash {trash_name} via Pose Teleport to dustbox...")
        self.stop_pose_tracking()

        drop_x, drop_y, drop_z = DUSTBOX_LOCATION
        req_str = f'name: "{trash_name}", position: {{x: {drop_x:.4f}, y: {drop_y:.4f}, z: {drop_z:.4f}}}'
        cmd = [
            'gz', 'service', '-s', '/world/office_room/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', req_str
        ]
        subprocess.run(cmd, capture_output=True)
        time.sleep(0.3)

        self.get_logger().info("[Step F] Opening gripper...")
        if not self.send_gripper_trajectory(GRIPPER_OPEN, 1.0):
            abort_sequence("Failed to open gripper at drop.")
            return response

        self.get_logger().info("[Step F] Returning to Home look-down patrol pose...")
        if not self.send_arm_trajectory(self.patrol_joints, 2.5):
            self.get_logger().error("Failed to return to Home patrol pose at end.")
            return response

        self.get_logger().info("Pick and place sequence successfully completed!")
        
        dummy_pose = PoseStamped()
        dummy_pose.header.stamp = self.get_clock().now().to_msg()
        response.plan.poses.append(dummy_pose)
        
        return response

    def destroy_node(self):
        self.stop_pose_tracking()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=16)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
