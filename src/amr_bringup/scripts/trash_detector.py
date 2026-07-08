#!/usr/bin/env python3
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs

class TrashDetector(Node):
    def __init__(self):
        super().__init__('trash_detector')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('optical_frame', 'oak_d_optical_link')
        
        # HSV threshold parameters for white trash detection
        self.declare_parameter('h_min', 15)
        self.declare_parameter('h_max', 45)
        self.declare_parameter('s_min', 2)
        self.declare_parameter('s_max', 20)
        self.declare_parameter('v_min', 120)
        self.declare_parameter('v_max', 200)
        self.declare_parameter('min_area', 20.0)
        self.declare_parameter('max_area', 5000.0)
        
        # Get parameter values
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        self.optical_frame = self.get_parameter('optical_frame').value
        
        self.get_logger().info(f"Subscribing to: \n  Image: {image_topic}\n  Depth: {depth_topic}\n  Info: {camera_info_topic}")
        
        # CV Bridge
        self.cv_bridge = CvBridge()
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.trash_pub = self.create_publisher(PoseStamped, '/detected_trash', 10)
        self.debug_image_pub = self.create_publisher(Image, '/trash_detector/debug_image', 10)
        
        # Message Filters for synchronization
        self.image_sub = message_filters.Subscriber(self, Image, image_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, camera_info_topic)
        
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.depth_sub, self.info_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        # For throttling to 5Hz
        self.last_inference_time = None
        
        # List to store coordinates (x, y, z) of already detected trash in map frame
        self.detected_trash_list = []
        
        self.get_logger().info("Trash Detector Node initialized successfully.")

    def sync_callback(self, img_msg, depth_msg, info_msg):
        # Throttle to 5Hz
        now = self.get_clock().now()
        if self.last_inference_time is not None:
            elapsed = (now - self.last_inference_time).nanoseconds / 1e9
            if elapsed < 0.19: # Slightly less than 0.2s to be safe
                return
        self.last_inference_time = now
        
        # Read parameters dynamically
        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        s_max = self.get_parameter('s_max').value
        v_min = self.get_parameter('v_min').value
        v_max = self.get_parameter('v_max').value
        min_area = self.get_parameter('min_area').value
        max_area = self.get_parameter('max_area').value
        
        try:
            # Convert image
            cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            
            # Convert depth
            if depth_msg.encoding == '16UC1':
                depth_array = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1').astype(np.float32) / 1000.0
            else:
                depth_array = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image or depth: {e}")
            return
        
        # HSV Thresholding
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([h_min, s_min, v_min], dtype=np.uint8)
        upper_bound = np.array([h_max, s_max, v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        
        # Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        debug_img = cv_image.copy()
        
        # Valid mask for depth
        valid_depth_mask = (depth_array > 0.1) & (depth_array < 10.0) & (~np.isnan(depth_array)) & (~np.isinf(depth_array))
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            
            # Bounding box
            x_rect, y_rect, w_rect, h_rect = cv2.boundingRect(contour)
            
            # Crop depth patch
            y_start = max(0, y_rect)
            y_end = min(depth_array.shape[0], y_rect + h_rect)
            x_start = max(0, x_rect)
            x_end = min(depth_array.shape[1], x_rect + w_rect)
            
            if y_start >= y_end or x_start >= x_end:
                continue
                
            depth_patch = depth_array[y_start:y_end, x_start:x_end]
            patch_valid_mask = valid_depth_mask[y_start:y_end, x_start:x_end]
            
            valid_depths = depth_patch[patch_valid_mask]
            if len(valid_depths) == 0:
                continue
                
            # Median of depths
            z = np.median(valid_depths)
            
            # Centroid of contour
            M = cv2.moments(contour)
            if M["m00"] != 0:
                u = M["m10"] / M["m00"]
                v = M["m01"] / M["m00"]
            else:
                u = x_rect + w_rect / 2.0
                v = y_rect + h_rect / 2.0
                
            # Camera intrinsics
            fx = info_msg.k[0]
            cx = info_msg.k[2]
            fy = info_msg.k[4]
            cy = info_msg.k[5]
            if fx == 0.0:
                fx = info_msg.p[0]
                cx = info_msg.p[2]
                fy = info_msg.p[5]
                cy = info_msg.p[6]
                
            if fx == 0.0 or fy == 0.0:
                self.get_logger().warn("Camera info has zero focal length!")
                continue
                
            # 3D coordinate in optical frame
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            
            # Prepare PoseStamped in optical frame
            pose_optical = PoseStamped()
            pose_optical.header.frame_id = self.optical_frame
            pose_optical.header.stamp = img_msg.header.stamp
            pose_optical.pose.position.x = float(x)
            pose_optical.pose.position.y = float(y)
            pose_optical.pose.position.z = float(z)
            pose_optical.pose.orientation.x = 0.0
            pose_optical.pose.orientation.y = 0.0
            pose_optical.pose.orientation.z = 0.0
            pose_optical.pose.orientation.w = 1.0
            
            # Transform to map frame
            try:
                # Use Time() to get the latest available transform to be robust against simulation lag
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    self.optical_frame,
                    Time(),
                    timeout=Duration(seconds=0.1)
                )
                pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_optical, transform)
                
                mx = pose_map.pose.position.x
                my = pose_map.pose.position.y
                mz = pose_map.pose.position.z
                
                # Check duplication
                is_duplicate = False
                for tx, ty, tz in self.detected_trash_list:
                    dist = math.sqrt((mx - tx)**2 + (my - ty)**2 + (mz - tz)**2)
                    if dist < 0.3:
                        is_duplicate = True
                        break
                
                # Draw on debug image
                if not is_duplicate:
                    color = (0, 255, 0) # Green for new
                    text = f"NEW {z:.2f}m"
                else:
                    color = (255, 0, 0) # Blue for already detected
                    text = f"DUP {z:.2f}m"
                    
                cv2.rectangle(debug_img, (x_rect, y_rect), (x_rect + w_rect, y_rect + h_rect), color, 2)
                cv2.putText(debug_img, text, (x_rect, y_rect - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                
                if not is_duplicate:
                    self.detected_trash_list.append((mx, my, mz))
                    self.trash_pub.publish(pose_map)
                    self.get_logger().info(f"Published NEW detected trash: map_pos=({mx:.3f}, {my:.3f}, {mz:.3f}), optical_pos=({x:.3f}, {y:.3f}, {z:.3f})")
            except Exception as e:
                self.get_logger().warn(f"Failed to transform optical frame to map: {e}")
                # Still draw it on debug image as unknown transform
                cv2.rectangle(debug_img, (x_rect, y_rect), (x_rect + w_rect, y_rect + h_rect), (0, 0, 255), 2)
                cv2.putText(debug_img, f"TF ERR {z:.2f}m", (x_rect, y_rect - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
        # Publish debug image
        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = img_msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
