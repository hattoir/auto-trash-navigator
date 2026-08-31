#!/usr/bin/env python3
"""可達範囲マッピング(TRAC-IK, /compute_ik経由)。

2026-08-24 実機HW確定(feature/hw-v2)により肩高さが 0.283m -> 0.168m
(11.5cm低下)したため、旧可達データ・接近オフセット・把持姿勢は全て
無効。本スクリプトで測り直す。

モード:
  floor  (既定) 床面 z=0.020 上を x:0.0-0.7 / y:-0.5-+0.5 を0.05刻みで走査。
         姿勢は pitch=90/75/60/45deg (水平からの角度、yawは対象方向に追従)
         および 姿勢自由(position-only IK) の5条件。
  height 指定した(x,y)点群について z=0.02-0.20 を0.02刻みで走査
         (Pre-grasp高さ決定用)。姿勢は pitch=60degで固定
         (床面スキャンで最も広い到達域を示した姿勢)。
"""
import sys
import math
import csv
import argparse
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ReachabilityMapper(Node):
    def __init__(self):
        super().__init__('reachability_mapper')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service to be available...')

        self.param_cli = self.create_client(SetParameters, '/move_group/set_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_group/set_parameters service...')
        self.get_logger().info('Services are available. Starting scan...')

    def set_position_only_ik(self, enable: bool):
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable)
        param = Parameter(name='robot_description_kinematics.arm.position_only_ik', value=val)
        req.parameters = [param]

        future = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None and res.results[0].successful:
            return True
        else:
            self.get_logger().error(f"Failed to set position_only_ik to {enable}")
            return False

    def check_ik(self, x, y, z, pitch_deg):
        """pitch_deg: 水平からの角度[deg](Noneなら姿勢自由=position-only)。
        yawは常にatan2(y,x)で対象方向へ追従させる(角度から見て自然な接近)。"""
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'grasp_link'
        req.ik_request.avoid_collisions = False

        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        if pitch_deg is None:
            # position-only IK: 姿勢は無視されるが、シード/ターゲットとして
            # 便宜的にstraight-downを渡す
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
        else:
            # 水平からの角度pitch_degを「Z+からの角度」に変換: 90+pitch_deg
            # (pitch_deg=90(真下)のとき180度=Z+から真反対、正しくstraight down)
            yaw = math.atan2(float(y), float(x))
            angle_from_zplus = (90.0 + pitch_deg) * math.pi / 180.0

            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(angle_from_zplus * 0.5)
            sp = math.sin(angle_from_zplus * 0.5)

            pose.pose.orientation.x = -sp * sy
            pose.pose.orientation.y = sp * cy
            pose.pose.orientation.z = cp * sy
            pose.pose.orientation.w = cp * cy

        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(sec=0, nanosec=100000000)  # 0.1s

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            return res.error_code.val == 1
        return False


def scan_floor(node, skip_free=False):
    z = 0.020

    x_coords = []
    curr_x = 0.7
    while curr_x >= -0.001:
        x_coords.append(round(curr_x, 2))
        curr_x -= 0.05

    y_coords = []
    curr_y = -0.5
    while curr_y <= 0.501:
        y_coords.append(round(curr_y, 2))
        curr_y += 0.05

    # (cond_id, pitch_deg or None, label)
    conditions = [
        (1, 90.0, "pitch=90deg (straight down)"),
        (2, 75.0, "pitch=75deg"),
        (3, 60.0, "pitch=60deg"),
        (4, 45.0, "pitch=45deg"),
        (5, None, "orientation free (position only)"),
    ]

    csv_rows = [['condition', 'pitch_deg', 'x', 'y', 'z', 'success']]
    summary = []
    all_grid_results = {}  # cond_val -> {x: {y: bool}}, used to build condition 5 as a union

    for cond_val, pitch_deg, cond_name in conditions:
        print(f"\nScanning for Condition {cond_val}: {cond_name}...")

        grid_results = {}
        success_xs = []
        success_ys = []

        if pitch_deg is None:
            # NOTE: MoveIt's /move_group/set_parameters can flip
            # robot_description_kinematics.arm.position_only_ik on the node,
            # but TRAC-IK only reads this at kinematics-plugin construction
            # time (move_group startup), not per-request -- confirmed
            # empirically: with the param toggled true, results were
            # byte-for-byte identical to Condition 1 (it silently fell back
            # to the straight-down seed orientation we pass for this case).
            # "orientation free" is therefore computed as the union of
            # Conditions 1-4 (reachable under ANY tested orientation implies
            # reachable when orientation is unconstrained) rather than by
            # re-querying IK. This is a lower bound on true position-only
            # reachability but avoids reporting a silently-wrong result.
            for x in x_coords:
                grid_results[x] = {}
                for y in y_coords:
                    success = any(all_grid_results[c][x][y] for c in all_grid_results)
                    grid_results[x][y] = success
                    csv_rows.append([cond_val, pitch_deg, x, y, z, 1 if success else 0])
                    if success:
                        success_xs.append(x)
                        success_ys.append(y)
        else:
            for x in x_coords:
                grid_results[x] = {}
                for y in y_coords:
                    success = node.check_ik(x, y, z, pitch_deg)
                    grid_results[x][y] = success
                    csv_rows.append([cond_val, pitch_deg, x, y, z, 1 if success else 0])
                    if success:
                        success_xs.append(x)
                        success_ys.append(y)

        all_grid_results[cond_val] = grid_results

        print(f"\n=== Reachability Map (Condition {cond_val}: {cond_name}) ===")
        print("Columns (Y): -0.5m to +0.5m (left to right, step 0.05m)")
        print("Rows (X): 0.7m down to 0.0m (top to bottom, step 0.05m)\n")

        print("      " + " ".join([f"{y: >5}" for y in y_coords]))
        for x in x_coords:
            row_str = f"{x: >4.2f} "
            for y in y_coords:
                val = "*" if grid_results[x][y] else "."
                row_str += f"   {val}  "
            print(row_str)

        print("\n=================================================================\n")

        if success_xs:
            min_x = min(success_xs)
            max_x = max(success_xs)
            max_abs_y = max([abs(y) for y in success_ys])
            print(f"Min successful X: {min_x:.3f} m")
            print(f"Max successful X: {max_x:.3f} m")
            print(f"Max successful absolute Y: {max_abs_y:.3f} m")
            summary.append((cond_val, cond_name, min_x, max_x, max_abs_y, len(success_xs)))
        else:
            print("No successful IK solutions found!")
            summary.append((cond_val, cond_name, None, None, None, 0))

    node.set_position_only_ik(False)

    csv_path = '/home/pakku/auto-trash-navigator/src/amr_bringup/scripts/reach_map.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_rows)
    node.get_logger().info(f"Results saved to {csv_path}")

    print("\n=== SUMMARY (floor scan, z=0.020) ===")
    print(f"{'Cond':<6}{'Label':<40}{'x_min':>8}{'x_max':>8}{'|y|max':>8}{'#pts':>6}")
    for cond_val, cond_name, min_x, max_x, max_abs_y, n in summary:
        if min_x is None:
            print(f"{cond_val:<6}{cond_name:<40}{'--':>8}{'--':>8}{'--':>8}{n:>6}")
        else:
            print(f"{cond_val:<6}{cond_name:<40}{min_x:>8.3f}{max_x:>8.3f}{max_abs_y:>8.3f}{n:>6}")


def scan_height(node, points):
    """points: list of (x, y) tuples. pitch固定60degで z=0.02-0.20を走査。"""
    pitch_deg = 60.0
    node.set_position_only_ik(False)

    z_coords = []
    curr_z = 0.02
    while curr_z <= 0.2001:
        z_coords.append(round(curr_z, 2))
        curr_z += 0.02

    csv_rows = [['x', 'y', 'z', 'success']]

    for (x, y) in points:
        print(f"\n=== Height scan at (x={x:.2f}, y={y:.2f}), pitch=60deg ===")
        results = {}
        for z in z_coords:
            success = node.check_ik(x, y, z, pitch_deg)
            results[z] = success
            csv_rows.append([x, y, z, 1 if success else 0])
        for z in z_coords:
            mark = "*" if results[z] else "."
            print(f"  z={z:.2f}  {mark}")
        succ_z = [z for z in z_coords if results[z]]
        if succ_z:
            print(f"  reachable z range: {min(succ_z):.2f} - {max(succ_z):.2f}")
        else:
            print("  no reachable z in range")

    node.set_position_only_ik(False)

    csv_path = '/home/pakku/auto-trash-navigator/src/amr_bringup/scripts/height_scan.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_rows)
    node.get_logger().info(f"Results saved to {csv_path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['floor', 'height'], default='floor')
    parser.add_argument('--points', type=str, default='',
                         help='height mode: "x1,y1;x2,y2;..." (m)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ReachabilityMapper()

    if args.mode == 'floor':
        scan_floor(node)
    else:
        points = []
        for pair in args.points.split(';'):
            if not pair.strip():
                continue
            xs, ys = pair.split(',')
            points.append((float(xs), float(ys)))
        if not points:
            points = [(0.30, 0.0)]
        scan_height(node, points)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
