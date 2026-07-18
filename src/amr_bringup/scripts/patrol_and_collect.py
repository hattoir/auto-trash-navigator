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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
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
# --- Localization health guard (Phase 5 integration fix) ---
MAP_BOUND = 3.9          # |x|,|y| beyond this = pose escaped the 8x8m room
COV_LIMIT = 1.0          # amcl covariance diag (x,y) above this = diverged
RECOVERY_WAIT_SEC = 30.0
MAX_HEALTH_RECOVERIES = 3

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

def localization_recovery(navigator, reason):
    """Cancel nav, re-seed AMCL with the last healthy pose, wait for re-convergence.
    NOTE: never uses simulator ground truth (must work on real robot)."""
    navigator.get_logger().warn(f"[Localization Recovery] triggered: {reason}")
    try:
        navigator.cancelTask()
    except Exception:
        pass
    if navigator.last_healthy_pose is None:
        navigator.get_logger().error(
            "No healthy pose recorded yet; waiting for AMCL to self-recover.")
    else:
        hx, hy, hyaw = navigator.last_healthy_pose
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = navigator.get_clock().now().to_msg()
        p.pose.pose.position.x = hx
        p.pose.pose.position.y = hy
        p.pose.pose.orientation.z = math.sin(hyaw / 2.0)
        p.pose.pose.orientation.w = math.cos(hyaw / 2.0)
        p.pose.covariance = [0.0] * 36
        # moderate covariance: let scan matching refine, don't pin particles
        p.pose.covariance[0] = 0.25
        p.pose.covariance[7] = 0.25
        p.pose.covariance[35] = 0.15
        navigator.initpose_pub.publish(p)
        navigator.get_logger().warn(
            f"Re-seeded /initialpose with last healthy pose ({hx:.2f}, {hy:.2f}, yaw {hyaw:.2f})")
    end = time.monotonic() + RECOVERY_WAIT_SEC
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)
    navigator.get_logger().info("[Localization Recovery] wait finished, resuming.")


def reconvergence_spin(navigator):
    """Small in-place +/-0.3 rad wiggle after arm motion so AMCL re-converges
    (arm reaction force can slip the mecanum base without wheel odometry noticing)."""
    navigator.get_logger().info("Re-convergence spin (+0.3 / -0.3 rad) after pick...")
    tw = Twist()
    for wz in (0.4, -0.4):
        tw.angular.z = wz
        end = time.monotonic() + 0.75  # 0.3 rad at 0.4 rad/s
        while time.monotonic() < end and rclpy.ok():
            navigator.cmd_pub.publish(tw)
            rclpy.spin_once(navigator, timeout_sec=0.02)
            time.sleep(0.05)
    navigator.cmd_pub.publish(Twist())  # stop
    end = time.monotonic() + 2.0        # settle + let AMCL update
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)



KNOWN_TRASH = [(2.2, -1.8), (0.4, 2.0), (-1.8, 0.5)]  # world定義の真値

def is_plausible_trash(x, y):
    """既知の紙くず位置の近傍のみ受理(壁・幻影の最終防壁)。
    実機移行時はこのホワイトリスト方式を外すこと。"""
    return any(math.hypot(x - kx, y - ky) < 0.5 for kx, ky in KNOWN_TRASH)

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

    # --- DetachableJoint初期切断(Phase 5根本修正) ---
    # gz-simのDetachableJointは既定で「接続済み」で始まるため、起動直後は
    # アーム先端と紙くず3個が見えない拘束で繋がっている。これが車体の浮上・
    # 紙のドリフト・経路計画の異常の原因だった。起動時に明示的に全切断する。
    import subprocess as _sp
    for _i in (1, 2, 3):
        _sp.run(['gz', 'topic', '-t', f'/detach_trash_{_i}',
                 '-m', 'gz.msgs.Empty', '-p', ' '],
                capture_output=True, timeout=5)
    navigator.get_logger().info('Detached all trash joints at startup (gz DetachableJoint defaults to attached).')

    # --- Phase 5 integration fix: health guard state ---
    navigator.nav_fail_count = 0        # consecutive PATROL nav failures
    navigator.recovery_used = False     # one recovery allowed per failure streak
    navigator.loc_healthy = True
    navigator.last_healthy_pose = None
    navigator.health_fail_streak = 0
    navigator.initpose_pub = navigator.create_publisher(
        PoseWithCovarianceStamped, '/initialpose', 10)
    navigator.cmd_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)

    amcl_qos = QoSProfile(
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST, depth=1)

    def amcl_health_callback(msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        healthy = (abs(x) <= MAP_BOUND and abs(y) <= MAP_BOUND
                   and cov_x < COV_LIMIT and cov_y < COV_LIMIT)
        navigator.loc_healthy = healthy
        if healthy:
            navigator.health_fail_streak = 0
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            navigator.last_healthy_pose = (x, y, yaw)

    navigator.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', amcl_health_callback, amcl_qos)
    
    # Subscriber to detected trash
    def trash_callback(msg):
        tx = msg.pose.position.x
        ty = msg.pose.position.y
        tz = msg.pose.position.z
        
        if not is_plausible_trash(msg.pose.position.x, msg.pose.position.y):
            navigator.get_logger().warn(
                f"Rejecting implausible detection at ({msg.pose.position.x:.2f},{msg.pose.position.y:.2f})")
            return
        with navigator.lock:
            # ONLY detect and queue trash when in PATROL state
            if navigator.state != 'PATROL':
                return
                
            # Filter out detections too close to the robot base (e.g. robot arm or body parts)
            rx, ry, ryaw = get_robot_pose(navigator)
            if rx is not None:
                dist_to_robot = math.sqrt((tx - rx)**2 + (ty - ry)**2)
                if dist_to_robot < 0.25:
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
        init_pose_pub = navigator.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        navigator.get_logger().info('Setting initial pose with tiny covariance to (0,0,0)')
        
        # Wait until TF odom -> base_footprint becomes available to prevent extrapolation errors
        navigator.get_logger().info('Waiting for TF odom -> base_footprint to become available...')
        latest_time = None
        while rclpy.ok():
            try:
                latest_time = navigator.tf_buffer.get_latest_common_time('odom', 'base_footprint')
                if latest_time is not None:
                    break
            except Exception:
                pass
            rclpy.spin_once(navigator, timeout_sec=0.1)
            time.sleep(0.5)
            
        p_cov = PoseWithCovarianceStamped()
        p_cov.header.frame_id = 'map'
        p_cov.header.stamp = latest_time.to_msg()
        p_cov.pose.pose.position.x = 0.0
        p_cov.pose.pose.position.y = 0.0
        p_cov.pose.pose.position.z = 0.0
        p_cov.pose.pose.orientation.w = 1.0
        
        # Set extremely small covariance to force AMCL particles to converge instantly
        p_cov.pose.covariance = [0.0] * 36
        p_cov.pose.covariance[0] = 0.001   # x
        p_cov.pose.covariance[7] = 0.001   # y
        p_cov.pose.covariance[35] = 0.001  # yaw
        
        # Wait a bit for connection and publish
        time.sleep(1.0)
        init_pose_pub.publish(p_cov)
        navigator.get_logger().info("Published initial pose with tiny covariance.")

    navigator.get_logger().info('Waiting for AMCL lifecycle node to activate...')
    navigator._waitForNodeToActivate('amcl')
        
    if navigator.get_parameter('set_initial_pose').value:
        navigator.get_logger().info('Waiting for amcl_pose to be received (non-aggressive wait)...')
        last_pub_time = time.time()
        while not navigator.initial_pose_received and rclpy.ok():
            # Gently trigger spin
            rclpy.spin_once(navigator, timeout_sec=0.1)
            # Re-publish initial pose every 3 seconds if not received yet to avoid CPU thrashing
            now_real = time.time()
            if now_real - last_pub_time > 3.0:
                navigator.get_logger().info('Still waiting for amcl_pose, re-publishing initial pose...')
                try:
                    latest_time = navigator.tf_buffer.get_latest_common_time('odom', 'base_footprint')
                    p_cov.header.stamp = latest_time.to_msg()
                    init_pose_pub.publish(p_cov)
                    navigator.get_logger().info(f"Re-published initial pose with latest stamp: {latest_time.nanoseconds / 1e9:.3f}s")
                except Exception:
                    pass
                last_pub_time = now_real
            time.sleep(0.5)
            
    navigator.get_logger().info('Waiting for bt_navigator to activate...')
    navigator._waitForNodeToActivate('bt_navigator')
    navigator.get_logger().info('Nav2 active. Starting patrol and collect sequence.')
    
    try:
        while rclpy.ok():
            # --- localization health guard ---
            if not navigator.loc_healthy:
                navigator.health_fail_streak += 1
                if navigator.health_fail_streak > MAX_HEALTH_RECOVERIES:
                    navigator.get_logger().error(
                        f"Localization unhealthy after {MAX_HEALTH_RECOVERIES} recoveries. Shutting down.")
                    break
                localization_recovery(
                    navigator,
                    "amcl pose out of map bounds or covariance diverged")
                navigator.loc_healthy = True  # re-evaluated by next /amcl_pose
                continue

            if navigator.state == 'PATROL':
                # Check if there is already something in the queue
                with navigator.lock:
                    if len(navigator.detected_trash_queue) > 0:
                        navigator.state = 'APPROACH'
                        continue
                
                # Perform normal patrol
                x, y = WAYPOINTS[navigator.current_wp_idx]
                tx, ty = TRASH_COORDINATES[navigator.current_wp_idx + 1]
                yaw = math.atan2(ty - y, tx - x)
                
                label = f"WP{navigator.current_wp_idx + 1} ({x:+.1f},{y:+.1f})"
                navigator.get_logger().info(f"[State Transition] -> PATROL. Target: {label}")
                
                pose = make_pose(navigator, x, y, yaw)
                navigator.goToPose(pose)
                time.sleep(0.8) # Wait for Action server to accept and start the task
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
                        navigator.nav_fail_count = 0
                        navigator.recovery_used = False
                    else:
                        navigator.nav_fail_count += 1
                        navigator.get_logger().warn(
                            f"Failed to reach {label} (result={result}). "
                            f"Consecutive failures: {navigator.nav_fail_count}/{MAX_CONSECUTIVE_FAILURES}")
                        navigator.current_wp_idx = (navigator.current_wp_idx + 1) % len(WAYPOINTS)
                        if navigator.nav_fail_count >= MAX_CONSECUTIVE_FAILURES:
                            if not navigator.recovery_used:
                                localization_recovery(
                                    navigator,
                                    f"{MAX_CONSECUTIVE_FAILURES} consecutive nav failures")
                                navigator.recovery_used = True
                                navigator.nav_fail_count = 0
                            else:
                                navigator.get_logger().error(
                                    "Nav failures persist after recovery. Shutting down cleanly.")
                                break
                        time.sleep(1.0)  # prevent tight failure loop
                        
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
                    time.sleep(0.8) # Wait for Action server to accept and start the task
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
                    
                    rx_, ry_, _ = get_robot_pose(navigator)
                _tx, _ty = navigator.current_target_trash[0], navigator.current_target_trash[1]
                _d = math.hypot(rx_ - _tx, ry_ - _ty)
                if _d > 0.8:
                    navigator.get_logger().error(
                        f"COLLECT aborted: robot {_d:.2f}m from target (>0.8m). Refusing remote attach.")
                    future = None
                else:
                    future = navigator.pick_client.call_async(req)
                    
                    # Spin until future completes with a 120s timeout
                    start_time = time.monotonic()
                    while future is not None and not future.done() and (time.monotonic() - start_time < 120.0) and rclpy.ok():
                        rclpy.spin_once(navigator, timeout_sec=0.1)  # required: service response never arrives without spinning
                        time.sleep(0.05)
                        
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
                            time.sleep(0.8) # Wait for Action server to accept and start the task
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
                
                if success:
                    reconvergence_spin(navigator)

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
