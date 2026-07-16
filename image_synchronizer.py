#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import copy

class ImageSynchronizer(Node):
    def __init__(self):
        super().__init__('image_synchronizer')
        
        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth_image_raw_vision', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw_sync', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth_image_raw_sync', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_sync', 10)
        
        # Stored latest messages
        self.latest_depth = None
        self.latest_info = None
        
        self.get_logger().info("ImageSynchronizer node started.")

    def depth_callback(self, msg):
        self.latest_depth = msg

    def info_callback(self, msg):
        self.latest_info = msg

    def image_callback(self, msg):
        if self.latest_depth is None or self.latest_info is None:
            return
            
        # Create deep copies to avoid modifying original message objects in memory
        sync_image = copy.deepcopy(msg)
        sync_depth = copy.deepcopy(self.latest_depth)
        sync_info = copy.deepcopy(self.latest_info)
        
        # Synchronize stamps to the rgb image stamp
        stamp = msg.header.stamp
        sync_image.header.stamp = stamp
        sync_depth.header.stamp = stamp
        sync_info.header.stamp = stamp
        
        # Publish synchronized topics
        self.image_pub.publish(sync_image)
        self.depth_pub.publish(sync_depth)
        self.info_pub.publish(sync_info)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSynchronizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
