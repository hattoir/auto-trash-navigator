#!/usr/bin/env python3
"""LiDAR自己ヒット除去フィルタ。

2026-08-24 実機HW確定(feature/hw-v2)で LiDAR-アーム間の距離関係が変わった
ため、旧来の一様min-range方式(0.28m未満を一律除去)では不十分になった。

【新配置(base_footprint基準)】
  LiDAR:     x=-0.160, y=+0.013, z=0.210, laser_frameにyaw+90°
  アーム取付: x=0.150,  y=0,      z=0.060 (joint1の水平位置。joint1軸はZ、
              アームはこの鉛直軸まわりに旋回するため、旋回角によらず
              水平方向にはこの点を中心に振れる)

【2026-09-01 ハード修正反映: 除去セクターの再算出】
  カメラマストを25mm下げた結果、OAK-D Lite光学中心が
  (0.182,0.135,0.197) → (0.189,0.135,0.180) に変更された。
  カメラ天端 z=0.1997 に対しLiDARスキャン面 z=0.210 で10.3mmの
  クリアランスがあり、もはやスキャン面を横切らないことがハード側の
  CAD干渉解析で確認済みのため、従来設けていたカメラ用除去セクターは
  撤廃する。

  アーム用セクターは、実際の関節角(home/look_down/grasp/drop_pose)で
  スキャン面 z=0.210 を横切る部位と位置を算出し直した:
    home(真上直立、全関節0)が最悪ケース: link2がスキャン面を横切り、
      水平位置は joint1=0 のため arm_base と同じ x=0.150, y=0。
      LiDAR(x=-0.160,y=0.013)からの距離・方位:
        distance = hypot(0.150-(-0.160), 0-0.013) = 0.310m
        bearing(base_link座標系) = atan2(0-0.013, 0.150-(-0.160)) = -2.40deg
      link2断面(0.036x0.032の対角線0.0482mを実効径として採用)による
      半値幅: half_angle = atan((0.0482/2)/0.310) = 4.44deg
    look_down/grasp/drop_pose(伸展姿勢)では link3/link6 がスキャン面を
      横切ることがあるが、これらはhomeでのlink2位置より水平方向に
      内側(LiDARから見て同じ方位帯の範囲内)に収まることを確認済みで、
      新たに幅を広げる要因にはならない。
    安全マージン ±1.5deg を加算: 半値幅 = 4.44 + 1.5 = 5.94deg
    中心方位(base_link座標系) = -2.40deg
    → base_link座標系でのセクター = [-2.40-5.94, -2.40+5.94]
                                   = [-8.34, 3.54] deg
    LiDAR自身のスキャン角度系への変換(laser_frameはbase_linkに対し
    yaw+90°回転して取り付けられているため、
    angle_in_lidar_frame = bearing_base_frame - 90deg):
      -8.34 - 90 = -98.34deg
       3.54 - 90 = -86.46deg
    除去セクター(LiDAR自身の角度系) = [-98.34, -86.46] deg
                                     = [-1.7164, -1.5090] rad

  距離閾値: 伸展時最大0.51m + 安全マージン0.09m = 0.60m(変更なし)。
  このセクター内・0.60m未満のみを除去する(セクター外や0.60m以遠は
  実際の障害物として温存し、探知能力を不必要に削らない)。
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# アーム自己ヒット除去セクター(LiDAR自身のスキャン角度系、rad)。
# 算出根拠は本ファイル冒頭のコメント参照。
ARM_SECTOR_MIN = math.radians(-98.34)
ARM_SECTOR_MAX = math.radians(-86.46)
ARM_SECTOR_RANGE_CUTOFF = 0.60  # m (伸展時最大0.51m + 安全マージン0.09m)


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(
            "LiDAR Self-Filter Node initialized. "
            f"Excluding arm sector [{math.degrees(ARM_SECTOR_MIN):.2f}, "
            f"{math.degrees(ARM_SECTOR_MAX):.2f}]deg below {ARM_SECTOR_RANGE_CUTOFF}m."
        )

    def callback(self, msg):
        filtered_msg = msg
        new_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            in_arm_sector = ARM_SECTOR_MIN <= angle <= ARM_SECTOR_MAX
            if in_arm_sector and r < ARM_SECTOR_RANGE_CUTOFF:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(r)
            angle += msg.angle_increment
        filtered_msg.ranges = new_ranges
        self.pub.publish(filtered_msg)


def main():
    rclpy.init()
    node = LidarFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
