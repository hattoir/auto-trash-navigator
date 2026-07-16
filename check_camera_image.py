#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CameraChecker(Node):
    def __init__(self):
        super().__init__('camera_checker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.get_logger().info('CameraChecker node started. Waiting for image...')
        self.received = False

    def listener_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}, encoding={msg.encoding}')
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == 'rgb8':
            data = data.reshape((msg.height, msg.width, 3))
        elif msg.encoding == 'bgr8':
            data = data.reshape((msg.height, msg.width, 3))
        
        # Calculate stats
        mean_val = np.mean(data, axis=(0, 1))
        min_val = np.min(data, axis=(0, 1))
        max_val = np.max(data, axis=(0, 1))
        self.get_logger().info(f'Image Stats (RGB/BGR channel-wise):')
        self.get_logger().info(f'  Min: {min_val}')
        self.get_logger().info(f'  Max: {max_val}')
        self.get_logger().info(f'  Mean: {mean_val}')
        
        # Count unique colors
        flat_data = data.reshape(-1, 3)
        unique_colors = np.unique(flat_data, axis=0)
        self.get_logger().info(f'Number of unique pixel colors in frame: {len(unique_colors)}')
        
        self.received = True

def main():
    rclpy.init()
    node = CameraChecker()
    while rclpy.ok() and not node.received:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
