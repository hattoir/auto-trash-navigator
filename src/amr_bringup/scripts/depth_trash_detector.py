#!/usr/bin/env python3
"""Depth-based trash detector.

RGBレンダリングが灰色化する環境(ogre2 + llvmpipe)でも動作する、
深度画像ベースのゴミ検出ノード。原理:
  1. 深度画像の各画素を3D点に逆投影し、base_footprint座標へ変換
  2. 「床より少し浮いた高さ(z: 0.005〜0.06m)にある小さな塊」を抽出
     -> 床(z=0)・壁/障害物(z>0.06が支配的)・遠景は自動的に除外される
  3. 塊の3D中心を map 座標へ変換し /detected_trash (PoseStamped) を配信

インターフェースは既存の trash_detector.py と互換:
  - 出力: /detected_trash (map座標のPoseStamped)
  - デバッグ: /trash_detector/debug_image (検出マスク可視化)
  - launchから渡される image_topic / HSVパラメータは受理するが未使用
"""
import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (register PointStamped transform)


class DepthTrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detector')
        # --- parameters (superset of the old color detector's, for launch compat) ---
        self.declare_parameter('use_sim_time_dummy', False)  # placeholder no-op
        self.declare_parameter('image_topic', '/camera/image_raw_sync')       # unused
        self.declare_parameter('depth_topic', '/camera/depth_image_raw_sync')
        self.declare_parameter('camera_info_topic', '/camera/camera_info_sync')
        for p, v in (('h_min', 0), ('h_max', 180), ('s_min', 0), ('s_max', 30),
                     ('v_min', 240), ('v_max', 255)):
            self.declare_parameter(p, v)  # accepted, unused (depth-based)
        self.declare_parameter('min_area', 5.0)      # blob pixel area lower bound
        self.declare_parameter('max_area', 2500.0)   # blob pixel area upper bound
        self.declare_parameter('z_min', 0.005)       # m above floor (lower)
        self.declare_parameter('z_max', 0.06)        # m above floor (upper)
        self.declare_parameter('x_max', 3.5)         # m ahead, detection range cap
        self.declare_parameter('optical_frame', '')  # '' = use depth header frame
        self.declare_parameter('detect_rate', 5.0)   # Hz throttle

        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.optical_frame_param = self.get_parameter('optical_frame').value
        self.min_period = 1.0 / max(float(self.get_parameter('detect_rate').value), 0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rays = None            # (H,W,3) unit-less ray dirs in optical frame
        self.info = None
        self.opt_to_base = None     # 4x4 static transform matrix
        self.detected_trash_list = []  # map-frame dedupe (0.3m)
        self.last_proc_stamp = None

        self.trash_pub = self.create_publisher(PoseStamped, '/detected_trash', 10)
        self.debug_pub = self.create_publisher(Image, '/trash_detector/debug_image', 2)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 5)
        self.get_logger().info(
            f"DepthTrashDetector started (depth={self.depth_topic}, "
            f"z-band {self.z_min}-{self.z_max}m, range<= {self.x_max}m)")

    # ------------------------------------------------------------------
    def info_cb(self, msg):
        if self.info is not None:
            return
        self.info = msg
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        h, w = msg.height, msg.width
        u = np.arange(w, dtype=np.float32)
        v = np.arange(h, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        # optical frame: x right, y down, z forward; depth = z distance
        self.rays = np.stack(((uu - cx) / fx, (vv - cy) / fy,
                              np.ones_like(uu)), axis=-1)
        self.get_logger().info(f"CameraInfo received ({w}x{h}, fx={fx:.1f})")

    def _lookup_static(self, optical_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                'base_footprint', optical_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f"TF base_footprint<-{optical_frame} not ready: {e}",
                                   throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]], dtype=np.float32)
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = (t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
        return T

    # ------------------------------------------------------------------
    def depth_cb(self, msg):
        if self.rays is None:
            return
        # throttle
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_proc_stamp is not None and stamp - self.last_proc_stamp < self.min_period:
            return
        self.last_proc_stamp = stamp

        optical_frame = self.optical_frame_param or msg.header.frame_id
        if self.opt_to_base is None:
            self.opt_to_base = self._lookup_static(optical_frame)
            if self.opt_to_base is None:
                return

        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) * 0.001
        else:
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}")
            return

        valid = np.isfinite(depth) & (depth > 0.2) & (depth < self.x_max + 1.0)
        # 3D in optical frame
        pts = self.rays * depth[..., None]                       # (H,W,3)
        # to base_footprint
        R, tvec = self.opt_to_base[:3, :3], self.opt_to_base[:3, 3]
        pb = pts @ R.T + tvec                                    # (H,W,3)
        zb = pb[..., 2]
        xb = pb[..., 0]
        band = valid & (zb > self.z_min) & (zb < self.z_max) \
            & (xb > 0.2) & (xb < self.x_max)
        mask = (band.astype(np.uint8)) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

        cnt, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        published = 0
        for i in range(1, cnt):
            area = stats[i, cv2.CC_STAT_AREA]
            if not (self.min_area <= area <= self.max_area):
                continue
            m = labels == i
            bx = float(np.median(xb[m]))
            by = float(np.median(pb[..., 1][m]))
            bz = float(np.median(zb[m]))
            # blob physical size sanity (<= 12cm extent)
            if (np.percentile(xb[m], 95) - np.percentile(xb[m], 5)) > 0.15:
                continue
            if not self._publish_map_pose(bx, by, bz, msg.header.stamp):
                continue
            published += 1
            self.get_logger().info(
                f"New trash detected (depth-based) base:({bx:.2f},{by:.2f},{bz:.3f}) "
                f"area={area}px")
        if self.debug_pub.get_subscription_count() > 0:
            dbg = Image()
            dbg.header = msg.header
            dbg.height, dbg.width = mask.shape
            dbg.encoding = 'mono8'
            dbg.step = mask.shape[1]
            dbg.data = mask.tobytes()
            self.debug_pub.publish(dbg)

    # ------------------------------------------------------------------
    def _publish_map_pose(self, bx, by, bz, stamp):
        ps = PointStamped()
        ps.header.frame_id = 'base_footprint'
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = bx, by, bz
        try:
            pm = self.tf_buffer.transform(ps, 'map', timeout=Duration(seconds=0.3))
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}", throttle_duration_sec=5.0)
            return False
        mx, my, mz = pm.point.x, pm.point.y, pm.point.z
        for tx, ty, tz in self.detected_trash_list:
            if math.hypot(mx - tx, my - ty) < 0.3:
                return False  # duplicate
        self.detected_trash_list.append((mx, my, mz))
        out = PoseStamped()
        out.header.frame_id = 'map'
        out.header.stamp = stamp
        out.pose.position.x, out.pose.position.y, out.pose.position.z = mx, my, mz
        out.pose.orientation.w = 1.0
        self.trash_pub.publish(out)
        self.get_logger().info(f"Published /detected_trash map:({mx:.2f},{my:.2f})")
        return True


def main():
    rclpy.init()
    node = DepthTrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
