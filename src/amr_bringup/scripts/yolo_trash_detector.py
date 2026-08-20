#!/usr/bin/env python3
"""YOLO-based trash detector (実機用パス)。

sim専用の depth_trash_detector.py とは別の、実機向け検出パイプライン:
  1. RGB画像をYOLO(ultralytics)に通し、紙くずのBBoxを検出
  2. BBox内の深度画像の中央値(0とNaNを除外)で距離を求め、
     カメラの内部パラメータで3D化(depth_trash_detector.pyと同じ
     ピンホールモデル: optical frame x=right, y=down, z=forward)
  3. 3D点をmap座標へ変換・重複抑制・地図境界棄却して /detected_trash
     へpublish -- このステップは trash_map_publish.py
     (depth_trash_detector.py の _publish_map_pose() を移植したもの)を
     そのまま流用し、車輪の再発明をしない

【重要】simのRGBカメラはogre2+llvmpipeレンダリングの制約で灰色描画
になり紙くずが写らないため、このノードはsimでは意味のある検出を
行えない(構造的な制約であり、本ノード自体のバグではない)。
学習は実写画像(~/trash_dataset/)で行い、このノードの検証は静止画/
rosbagベースで実施すること。simでの回帰確認は既存の
depth_trash_detector.py で行う。

インターフェース(depth_trash_detector.pyと合わせている):
  - 購読: image_topic(RGB) / depth_topic / camera_info_topic
  - 出力: /detected_trash (map座標のPoseStamped)
  - デバッグ: /trash_detector/debug_image (BBox+信頼度を描画したRGB画像)
"""
import os

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (register PointStamped transform)

from trash_map_publish import TrashMapPublisher

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def default_model_path():
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('amr_vision')
        return os.path.join(share_dir, 'models', 'trash_yolov8n.pt')
    except Exception:
        return ''


class YoloTrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detector')
        self.declare_parameter('image_topic', '/camera/image_raw_sync')
        self.declare_parameter('depth_topic', '/camera/depth_image_raw_sync')
        self.declare_parameter('camera_info_topic', '/camera/camera_info_sync')
        self.declare_parameter('optical_frame', '')  # '' = use image header frame
        self.declare_parameter('model_path', default_model_path())
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('detect_rate', 5.0)   # Hz throttle
        self.declare_parameter('depth_range_min', 0.1)   # m, sanity range
        self.declare_parameter('depth_range_max', 4.0)   # m, sanity range

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.optical_frame_param = self.get_parameter('optical_frame').value
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.min_period = 1.0 / max(float(self.get_parameter('detect_rate').value), 0.1)
        self.depth_range_min = float(self.get_parameter('depth_range_min').value)
        self.depth_range_max = float(self.get_parameter('depth_range_max').value)

        if YOLO is None:
            self.get_logger().error(
                "ultralytics is not installed; yolo_trash_detector.py cannot run inference. "
                "Install with: pip3 install --break-system-packages ultralytics")
            self.model = None
        elif not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(f"YOLO model not found at '{self.model_path}'.")
            self.model = None
        else:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"Loaded YOLO model: {self.model_path}")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.info = None
        self.last_depth = None       # (H,W) float32 meters
        self.last_depth_stamp = None
        self.last_proc_stamp = None

        self.trash_pub = self.create_publisher(PoseStamped, '/detected_trash', 10)
        self.debug_pub = self.create_publisher(Image, '/trash_detector/debug_image', 2)
        self.map_publisher = TrashMapPublisher(self, self.tf_buffer, self.trash_pub)

        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 5)
        self.create_subscription(Image, self.image_topic, self.image_cb, 5)
        self.get_logger().info(
            f"YoloTrashDetector started (image={self.image_topic}, depth={self.depth_topic}, "
            f"conf_thresh={self.confidence_threshold}, rate={1.0/self.min_period:.1f}Hz)")

    # ------------------------------------------------------------------
    def info_cb(self, msg):
        if self.info is not None:
            return
        self.info = msg
        self.get_logger().info(f"CameraInfo received ({msg.width}x{msg.height})")

    # ------------------------------------------------------------------
    def depth_cb(self, msg):
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width).astype(np.float32) * 0.001
        else:
            self.get_logger().error(f"Unsupported depth encoding: {msg.encoding}",
                                     throttle_duration_sec=10.0)
            return
        self.last_depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        self.last_depth_stamp = msg.header.stamp

    # ------------------------------------------------------------------
    def image_cb(self, msg):
        if self.model is None or self.info is None or self.last_depth is None:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_proc_stamp is not None and stamp - self.last_proc_stamp < self.min_period:
            return
        self.last_proc_stamp = stamp

        rgb = self._decode_rgb(msg)
        if rgb is None:
            return

        depth = self.last_depth
        if depth.shape[:2] != rgb.shape[:2]:
            # 深度とRGBの解像度が異なる場合はBBox座標を深度側にスケールする
            scale_x = depth.shape[1] / rgb.shape[1]
            scale_y = depth.shape[0] / rgb.shape[0]
        else:
            scale_x = scale_y = 1.0

        results = self.model.predict(rgb, conf=self.confidence_threshold, verbose=False)
        boxes = results[0].boxes if results else None

        optical_frame = self.optical_frame_param or msg.header.frame_id
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]

        debug_img = rgb.copy()
        n_detections = 0
        if boxes is not None:
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])

                dx1, dy1 = int(x1 * scale_x), int(y1 * scale_y)
                dx2, dy2 = int(x2 * scale_x), int(y2 * scale_y)
                dx1, dy1 = max(0, dx1), max(0, dy1)
                dx2 = min(depth.shape[1], max(dx2, dx1 + 1))
                dy2 = min(depth.shape[0], max(dy2, dy1 + 1))
                patch = depth[dy1:dy2, dx1:dx2]
                valid = patch[(patch > self.depth_range_min) & (patch < self.depth_range_max)]
                if valid.size == 0:
                    cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    continue
                d = float(np.median(valid))

                u = (x1 + x2) / 2.0
                v = (y1 + y2) / 2.0
                x_opt = (u - cx) / fx * d
                y_opt = (v - cy) / fy * d
                z_opt = d

                ps = PointStamped()
                ps.header.frame_id = optical_frame
                ps.header.stamp = rclpy.time.Time().to_msg()
                ps.point.x, ps.point.y, ps.point.z = x_opt, y_opt, z_opt
                published = self.map_publisher.publish_from_point_stamped(ps)

                color = (0, 255, 0) if published else (0, 200, 200)
                cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(debug_img, f"{conf:.2f} d={d:.2f}m", (int(x1), max(0, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                n_detections += 1
                if published:
                    self.get_logger().info(
                        f"New trash detected (YOLO) conf={conf:.2f} depth={d:.2f}m "
                        f"bbox=({x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f})")

        self._publish_debug_image(debug_img, msg.header)

    # ------------------------------------------------------------------
    def _decode_rgb(self, msg):
        if msg.encoding in ('rgb8', 'bgr8'):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr
        else:
            self.get_logger().error(f"Unsupported image encoding: {msg.encoding}",
                                     throttle_duration_sec=10.0)
            return None

    def _publish_debug_image(self, bgr_img, header):
        dbg = Image()
        dbg.header = header
        dbg.height, dbg.width = bgr_img.shape[:2]
        dbg.encoding = 'bgr8'
        dbg.step = bgr_img.shape[1] * 3
        dbg.data = bgr_img.tobytes()
        self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = YoloTrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
