#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilterNode(Node):
    def __init__(self):
        super().__init__('lidar_filter_node')
        self.sub = self.create_subscription(LaserScan, '/scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info("LiDAR Self-Filter Node initialized. Filtering ranges < 0.28m.")

    def callback(self, msg):
        filtered_msg = msg
        new_ranges = []
        for r in msg.ranges:
            if r < 0.28:
                new_ranges.append(float('inf'))
            else:
                new_ranges.append(r)
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
