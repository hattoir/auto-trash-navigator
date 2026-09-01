#!/usr/bin/env python3
"""diff-drive 開ループ真値試験。
/cmd_velの linear.x / angular.z / linear.y を各3秒印加し、
gz真値の変位方向(x,yawは正方向、yは差動駆動のため不動)と
/odomとの差(0.15以内)を確認する。"""
import subprocess
import time
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def get_gz_pose(model='visual_amr'):
    out = subprocess.run(['gz', 'model', '-m', model, '-p'],
                          capture_output=True, text=True, timeout=10).stdout
    lines = out.splitlines()
    for i, line in enumerate(lines):
        if 'Pose [' in line:
            xyz = [float(v) for v in lines[i + 1].strip().strip('[]').split()]
            rpy = [float(v) for v in lines[i + 2].strip().strip('[]').split()]
            return xyz[0], xyz[1], rpy[2]
    raise RuntimeError(f"Could not parse gz pose from: {out}")


class OdomListener(Node):
    def __init__(self):
        super().__init__('diffdrive_openloop_verifier')
        self.pose = None
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_twist(self, lin_x, lin_y, ang_z, duration):
        tw = Twist()
        tw.linear.x = lin_x
        tw.linear.y = lin_y
        tw.angular.z = ang_z
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())  # stop
        self.spin_for(0.5)


def main():
    rclpy.init()
    node = OdomListener()
    node.get_logger().info('Waiting for first /odom message...')
    while node.pose is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.spin_for(1.0)

    # (name, lin_x, lin_y, ang_z, expect: 'positive' | 'zero')
    tests = [
        ('+x', 0.20, 0.0, 0.0, 'positive'),
        ('+yaw', 0.0, 0.0, 0.5, 'positive'),
        ('+y', 0.0, 0.20, 0.0, 'zero'),
    ]

    results = []
    for name, lx, ly, az, expect in tests:
        gz_before = get_gz_pose()
        odom_before = node.pose
        node.run_twist(lx, ly, az, 3.0)
        time.sleep(0.3)
        gz_after = get_gz_pose()
        odom_after = node.pose

        dgz = (gz_after[0] - gz_before[0], gz_after[1] - gz_before[1],
               gz_after[2] - gz_before[2])
        dod = (odom_after[0] - odom_before[0], odom_after[1] - odom_before[1],
               odom_after[2] - odom_before[2])

        print(f"\n=== {name} test (expect={expect}) ===")
        print(f"  gz_before  = ({gz_before[0]:.4f}, {gz_before[1]:.4f}, {math.degrees(gz_before[2]):.2f}deg)")
        print(f"  gz_after   = ({gz_after[0]:.4f}, {gz_after[1]:.4f}, {math.degrees(gz_after[2]):.2f}deg)")
        print(f"  gz_delta   = ({dgz[0]:.4f}, {dgz[1]:.4f}, {math.degrees(dgz[2]):.2f}deg)")
        print(f"  odom_before= ({odom_before[0]:.4f}, {odom_before[1]:.4f}, {math.degrees(odom_before[2]):.2f}deg)")
        print(f"  odom_after = ({odom_after[0]:.4f}, {odom_after[1]:.4f}, {math.degrees(odom_after[2]):.2f}deg)")
        print(f"  odom_delta = ({dod[0]:.4f}, {dod[1]:.4f}, {math.degrees(dod[2]):.2f}deg)")

        if name == '+x':
            gz_val, od_val = dgz[0], dod[0]
        elif name == '+yaw':
            gz_val, od_val = dgz[2], dod[2]
        else:  # +y
            # 差動駆動でx/y世界座標のどちらに出るかは、その時点の姿勢yawに
            # 依存する(ロボットローカルyコマンドに対しては本来「動かない」
            # ことが正しく、ワールド変位ベクトルの大きさ(x,yどちらの成分か
            # ではなくノルム)がゼロに近いかで判定する)。
            gz_val = math.hypot(dgz[0], dgz[1])
            od_val = math.hypot(dod[0], dod[1])

        diff = abs(gz_val - od_val)
        if expect == 'positive':
            direction_ok = gz_val > 0
        else:
            direction_ok = abs(gz_val) < 0.02  # ほぼ不動(数値誤差程度)
        diff_ok = diff <= 0.15
        print(f"  gz_val={gz_val:.4f}  odom_val={od_val:.4f}  |diff|={diff:.4f}  "
              f"{'positive' if expect=='positive' else 'near-zero'}={'OK' if direction_ok else 'FAIL'}  "
              f"diff<=0.15={'OK' if diff_ok else 'FAIL'}")
        results.append((name, expect, gz_val, od_val, diff, direction_ok, diff_ok))

    print("\n=== SUMMARY ===")
    all_ok = True
    for name, expect, gz_val, od_val, diff, direction_ok, diff_ok in results:
        ok = direction_ok and diff_ok
        all_ok &= ok
        print(f"{name} (expect={expect}): gz={gz_val:.4f} odom={od_val:.4f} diff={diff:.4f} -> {'PASS' if ok else 'FAIL'}")
    print(f"\nOVERALL: {'PASS' if all_ok else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
