#!/usr/bin/env python3
"""HW-v2 ピック単体検証。
(0.40,0.00) / (0.40,+0.08) / (0.40,-0.08) [base_footprint相対] に各5回、
計15試行。ロボットはワールド原点(0,0,yaw=0)に静止した状態のまま、
gz teleportで紙くずをテスト座標へ配置し、/pick_trashを呼び出す。
各試行後、joint2がhome(0)へ復帰していることを/joint_statesで確認する。
"""
import math
import subprocess
import time
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import JointState

TEST_POINTS = [
    ("center", 0.40, 0.00),
    ("plus_y", 0.40, 0.08),
    ("minus_y", 0.40, -0.08),
]
TRIALS_PER_POINT = 5
JOINT2_HOME_TOL = 0.05  # rad


def teleport_model(name, x, y, z=0.05):
    req_str = f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
    cmd = [
        'gz', 'service', '-s', '/world/office_room/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req_str
    ]
    subprocess.run(cmd, capture_output=True, timeout=5.0)


class PickUnitTestNode(Node):
    def __init__(self):
        super().__init__('pick_unit_test_node', parameter_overrides=[
            rclpy.Parameter('use_sim_time', value=True)
        ])
        self.cli = self.create_client(GetPlan, '/pick_trash')
        self.latest_joint_state = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /pick_trash service...')

    def js_cb(self, msg):
        self.latest_joint_state = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_joint2(self):
        self.spin_for(0.3)
        if self.latest_joint_state is None:
            return None
        try:
            idx = list(self.latest_joint_state.name).index('joint2')
            return self.latest_joint_state.position[idx]
        except (ValueError, IndexError):
            return None

    def call_pick(self, x, y, z=0.02, timeout_s=120.0):
        req = GetPlan.Request()
        req.goal = PoseStamped()
        req.goal.header.frame_id = 'base_footprint'
        req.goal.pose.position.x = float(x)
        req.goal.pose.position.y = float(y)
        req.goal.pose.position.z = float(z)
        req.goal.pose.orientation.w = 1.0
        future = self.cli.call_async(req)
        start = time.monotonic()
        while not future.done() and (time.monotonic() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return False
        res = future.result()
        return res is not None and len(res.plan.poses) > 0


def main():
    rclpy.init()
    node = PickUnitTestNode()

    results = []
    trash_cycle = ['paper_trash_1', 'paper_trash_2', 'paper_trash_3']
    trial_idx = 0

    for label, x, y in TEST_POINTS:
        for trial in range(1, TRIALS_PER_POINT + 1):
            trash_name = trash_cycle[trial_idx % len(trash_cycle)]
            trial_idx += 1

            # Teleport trash near the test point (robot at world origin,
            # so base_footprint coords == world coords numerically).
            teleport_model(trash_name, x, y, 0.05)
            node.spin_for(1.0)

            print(f"\n=== {label} trial {trial}/{TRIALS_PER_POINT}: "
                  f"target=({x:.2f},{y:.2f}) trash={trash_name} ===")
            success = node.call_pick(x, y, 0.02)
            print(f"  pick result: {'SUCCESS' if success else 'FAIL'}")

            node.spin_for(1.0)
            joint2 = node.get_joint2()
            joint2_ok = joint2 is not None and abs(joint2) <= JOINT2_HOME_TOL
            print(f"  joint2 after sequence: {joint2} -> "
                  f"{'HOME_OK' if joint2_ok else 'NOT_HOME'}")

            results.append({
                'label': label, 'trial': trial, 'x': x, 'y': y,
                'trash': trash_name, 'success': success,
                'joint2': joint2, 'joint2_ok': joint2_ok,
            })

    print("\n=== SUMMARY ===")
    total_success = sum(1 for r in results if r['success'])
    total_joint2_ok = sum(1 for r in results if r['joint2_ok'])
    for r in results:
        print(f"{r['label']:>8} trial{r['trial']} target=({r['x']:.2f},{r['y']:.2f}) "
              f"trash={r['trash']:<15} pick={'OK' if r['success'] else 'FAIL':<4} "
              f"joint2={r['joint2']} ({'HOME' if r['joint2_ok'] else 'NOT_HOME'})")
    print(f"\nTotal pick success: {total_success}/{len(results)}")
    print(f"Total joint2-home-ok: {total_joint2_ok}/{len(results)}")
    print(f"PASS criterion (>=12/15 pick success): "
          f"{'PASS' if total_success >= 12 else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if total_success >= 12 else 1)


if __name__ == '__main__':
    main()
