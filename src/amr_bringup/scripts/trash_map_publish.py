#!/usr/bin/env python3
"""検出したゴミの3D位置をmap座標へ変換・重複抑制・地図境界棄却して
publishする共通ロジック。

depth_trash_detector.py の _publish_map_pose() をそのまま移植した
ものであり、車輪の再発明を避けるために新規検出器(yolo_trash_detector.py
など)から共有する。depth_trash_detector.py 自体は検証済み・完成済みの
simの正規パスであるため、回帰リスクを避けるためにこのファイルからは
一切変更していない(重複コードになっているが、それぞれ独立して動く
検出器が同じロジックを使うための意図的な選択)。
"""
import math

from geometry_msgs.msg import PoseStamped, PointStamped
from rclpy.duration import Duration
from rclpy.time import Time


class TrashMapPublisher:
    """map座標変換・0.3m重複抑制・地図境界(±3.6m)棄却を行い、
    /detected_trash (PoseStamped) を配信するヘルパー。
    検出器ノード側で1インスタンス保持し、検出のたびに
    publish_from_base_footprint() または publish_from_point_stamped() を
    呼び出す。
    """

    def __init__(self, node, tf_buffer, trash_pub, dedupe_radius_m=0.3, map_bound_m=3.6):
        self.node = node
        self.tf_buffer = tf_buffer
        self.trash_pub = trash_pub
        self.dedupe_radius_m = dedupe_radius_m
        self.map_bound_m = map_bound_m
        self.detected_trash_list = []  # map-frame dedupe

    def publish_from_point_stamped(self, point_stamped):
        """任意のframe_idのPointStampedをmapへ変換してpublishを試みる。
        成功時True、TF失敗/重複/地図外でスキップした場合Falseを返す。"""
        try:
            pm = self.tf_buffer.transform(point_stamped, 'map', timeout=Duration(seconds=0.3))
        except Exception as e:
            self.node.get_logger().warn(f"TF to map failed: {e}", throttle_duration_sec=5.0)
            return False
        return self._publish_map_point(pm.point.x, pm.point.y, pm.point.z)

    def publish_from_base_footprint(self, bx, by, bz):
        """base_footprint座標(x,y,z)をmapへ変換してpublishを試みる。"""
        ps = PointStamped()
        ps.header.frame_id = 'base_footprint'
        ps.header.stamp = Time().to_msg()
        ps.point.x, ps.point.y, ps.point.z = bx, by, bz
        return self.publish_from_point_stamped(ps)

    def _publish_map_point(self, mx, my, mz):
        if abs(mx) > self.map_bound_m or abs(my) > self.map_bound_m:
            return False  # 地図外=幻影
        for tx, ty, tz in self.detected_trash_list:
            if math.hypot(mx - tx, my - ty) < self.dedupe_radius_m:
                return False  # duplicate
        self.detected_trash_list.append((mx, my, mz))
        out = PoseStamped()
        out.header.frame_id = 'map'
        out.header.stamp = Time().to_msg()
        out.pose.position.x, out.pose.position.y, out.pose.position.z = mx, my, mz
        out.pose.orientation.w = 1.0
        self.trash_pub.publish(out)
        self.node.get_logger().info(f"Published /detected_trash map:({mx:.2f},{my:.2f})")
        return True
