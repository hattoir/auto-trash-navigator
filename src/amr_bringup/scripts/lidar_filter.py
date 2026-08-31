#!/usr/bin/env python3
"""LiDAR自己ヒット除去フィルタ。

2026-08-24 実機HW確定(feature/hw-v2)で LiDAR-アーム間の距離関係が変わった
ため、旧来の一様min-range方式(0.28m未満を一律除去)では不十分になった。

【新配置(base_footprint基準)】
  LiDAR:     x=-0.160, y=+0.013, z=0.210, laser_frameにyaw+90°
  アーム取付: x=0.150,  y=0,      z=0.060 (joint1の水平位置。joint1軸はZ、
              アームはこの鉛直軸まわりに旋回するため、旋回角によらず
              水平方向にはこの点を中心に振れる)

  LiDARからアーム取付点までの水平距離: 0.31m(静止時)
  アームを前方へ伸ばした場合の距離   : 0.51m(手先0.20m分の伸展を仮定)
  いずれもLiDARからはほぼ同じ方位に見える(y方向のオフセットが0.013mと
  小さいため、伸展による方位変化は±0.5°程度に留まる)。
  → 「距離によらず除去が必要な範囲が変わる」問題ではなく、
    「アームは常にLiDARから見てほぼ同じ狭い方位(セクター)にいる」
    という性質を使い、min-rangeではなく角度セクター+距離閾値で除去する。

【除去セクターの算出根拠】(tools/以下に検証コードなし、本コメントに算出過程を明記)
  1. 方位(base_link座標系):
       arm_base=(0.150,0) → bearing=atan2(0-0.013, 0.150-(-0.160))  = -2.401deg
       arm_ext =(0.350,0) → bearing=atan2(0-0.013, 0.350-(-0.160))  = -1.460deg
       (arm_extは arm_base から+X方向に0.20m伸ばした点。LiDARからの距離が
        ちょうど0.51mになり、仕様の「伸ばすと0.51m」と一致する)
     中心方位 = (-2.401 + -1.460)/2 = -1.931deg (base_link座標系)
     伸展による方位ぶれの半値幅 = |(-2.401)-(-1.460)|/2 = 0.471deg
  2. アーム太さによる角度幅:
       so101_arm.xacro の link2 断面(0.036 x 0.032、アーム中で最も太い
       ピッチ軸リンク)の対角線 = hypot(0.036,0.032) = 0.0482m を
       「実効アーム径」として採用(どの向きで倒れても包含できる保守値)。
       最も角度幅が大きくなるのは距離が最も近い時(0.31m)なので、そこで評価:
       half_angle = atan((0.0482/2) / 0.31) = 4.438deg
  3. 合計半値幅 = 4.438(太さ) + 0.471(伸展ぶれ) + 0.5(安全マージン) = 5.409deg
     → 全幅 約10.8deg (必要最小限。従来のmin-range一律除去(360°全周)
       に比べ大幅に限定)
  4. LiDAR自身のスキャン角度系への変換:
       laser_frameはbase_linkに対しyaw+90°回転して取り付けられているため、
       base_link座標系の方位からLiDAR自身の角度系への変換は
       angle_in_lidar_frame = bearing_base_frame - 90deg
       中心: -1.931 - 90 = -91.931deg
     除去セクター(LiDAR自身の角度系) = [-97.340, -86.522] deg
                                     = [-1.6989, -1.5101] rad

  距離閾値: 伸展時最大0.51m + 安全マージン0.09m = 0.60m。
  このセクター内・0.60m未満のみを除去する(セクター外や0.60m以遠は
  実際の障害物として温存し、探知能力を不必要に削らない)。

【2026-09-01 追記: OAK-Dカメラの自己ヒット(統合検証で新規発見)】
  上記のアームセクターだけでは不十分だった。統合検証(full_autonomy,
  巡回姿勢=全関節0)で collision_monitor が常時「接近中」と誤判定を
  繰り返し、Nav2が実質的に前進できない不具合が発生した。実際の
  /scan(フィルタ後)を調べたところ、角度-76.7°〜-63.7°・距離0.34〜0.37m
  に持続的な近距離ヒットがあり、アームセクター([-97.3,-86.5]deg)の
  外側だった。TF実測(lidar_link -> oak_d_link)で原因を特定:
    位置(lidar_link座標系) = (0.1220, -0.3420) → 距離0.3631m,
    角度atan2(-0.342,0.122) = -70.367deg
  これは実測ヒットの角度・距離と一致し、自己ヒットの正体はアームでは
  なくOAK-Dカメラ本体と判明した。HW-v2でカメラ位置を(0.182,0.135,0.197)
  に変更した結果、LiDARスキャン面(z=0.210)にカメラ筐体上端が非常に
  近くなり干渉するようになった(旧カメラ位置z=0.1では発生しなかった)。
  カメラは車体に剛結され姿勢によらず位置が変わらないため、伸展を
  考慮する必要はなく、単一の固定角度・固定距離の点として扱える。

  除去セクターの算出:
    中心角度: -70.367deg (lidar_link座標系、TF実測)
    距離: 0.3631m
    カメラ筐体(oak_d_linkのvisualボックス0.03x0.1x0.03、最長辺0.10mを
    保守的な水平方向サイズとして採用):
      half_angle = atan((0.10/2) / 0.3631) = 7.840deg
    安全マージン0.5degを加え、半値幅 = 8.340deg
    → セクター = [-78.708, -62.027] deg
    距離閾値 = 0.3631 + 0.10(安全マージン) = 0.463m
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# アーム自己ヒット除去セクター(LiDAR自身のスキャン角度系、rad)。
# 算出根拠は本ファイル冒頭のコメント参照。
ARM_SECTOR_MIN = math.radians(-97.340)
ARM_SECTOR_MAX = math.radians(-86.522)
ARM_SECTOR_RANGE_CUTOFF = 0.60  # m (伸展時最大0.51m + 安全マージン0.09m)

# OAK-Dカメラ自己ヒット除去セクター(LiDAR自身のスキャン角度系、rad)。
# 算出根拠は本ファイル冒頭の追記コメント参照。
CAMERA_SECTOR_MIN = math.radians(-78.708)
CAMERA_SECTOR_MAX = math.radians(-62.027)
CAMERA_SECTOR_RANGE_CUTOFF = 0.463  # m


class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info(
            "LiDAR Self-Filter Node initialized. "
            f"Excluding arm sector [{math.degrees(ARM_SECTOR_MIN):.1f}, "
            f"{math.degrees(ARM_SECTOR_MAX):.1f}]deg below {ARM_SECTOR_RANGE_CUTOFF}m, "
            f"camera sector [{math.degrees(CAMERA_SECTOR_MIN):.1f}, "
            f"{math.degrees(CAMERA_SECTOR_MAX):.1f}]deg below {CAMERA_SECTOR_RANGE_CUTOFF}m."
        )

    def callback(self, msg):
        filtered_msg = msg
        new_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            in_arm_sector = ARM_SECTOR_MIN <= angle <= ARM_SECTOR_MAX
            in_camera_sector = CAMERA_SECTOR_MIN <= angle <= CAMERA_SECTOR_MAX
            if in_arm_sector and r < ARM_SECTOR_RANGE_CUTOFF:
                new_ranges.append(float('inf'))
            elif in_camera_sector and r < CAMERA_SECTOR_RANGE_CUTOFF:
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
