#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import re
import time
import math

class EvidenceCollector(Node):
    def __init__(self):
        super().__init__('evidence_collector')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
        self.latest_scan_min = float('nan')
        self.latest_amcl_x = float('nan')
        self.latest_amcl_y = float('nan')
        self.latest_amcl_cov_x = float('nan')
        self.latest_amcl_cov_y = float('nan')
        
        self.log_file = open('/home/pakku/auto-trash-navigator/evidence_log.csv', 'w')
        self.log_file.write("sim_time,scan_min,amcl_x,amcl_y,cov_x,cov_y,gt_x,gt_y,gt_yaw,error_dist\n")
        self.log_file.flush()
        
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("EvidenceCollector started.")

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            self.latest_scan_min = min(valid_ranges)
        else:
            self.latest_scan_min = float('nan')

    def amcl_callback(self, msg):
        self.latest_amcl_x = msg.pose.pose.position.x
        self.latest_amcl_y = msg.pose.pose.position.y
        self.latest_amcl_cov_x = msg.pose.covariance[0]  # x-x variance
        self.latest_amcl_cov_y = msg.pose.covariance[7]  # y-y variance

    def get_gazebo_pose(self):
        try:
            res = subprocess.run(
                ['gz', 'model', '-m', 'visual_amr', '-p'],
                capture_output=True, text=True, timeout=10.0
            )
            output = res.stdout
            # Extract Pose [ XYZ (m) ] [ RPY (rad) ]:
            # [0.146691 0.000003 0.262857]
            # [0.000005 -0.589897 0.000008]
            lines = output.split('\n')
            xyz_line = ""
            rpy_line = ""
            for idx, line in enumerate(lines):
                if "Pose [ XYZ (m) ]" in line:
                    xyz_line = lines[idx+1].strip()
                    rpy_line = lines[idx+2].strip()
                    break
            
            xyz_match = re.findall(r'[-+]?\d*\.\d+|\d+', xyz_line)
            rpy_match = re.findall(r'[-+]?\d*\.\d+|\d+', rpy_line)
            
            if len(xyz_match) >= 3 and len(rpy_match) >= 3:
                x = float(xyz_match[0])
                y = float(xyz_match[1])
                yaw = float(rpy_match[2])
                return x, y, yaw
        except Exception as e:
            self.get_logger().error(f"Error getting gazebo pose: {e}")
        return float('nan'), float('nan'), float('nan')

    def timer_callback(self):
        # Get ROS simulation time
        now = self.get_clock().now()
        sec, nsec = now.seconds_nanoseconds()
        sim_time = sec + nsec * 1e-9
        
        gt_x, gt_y, gt_yaw = self.get_gazebo_pose()
        
        error_dist = float('nan')
        if not math.isnan(self.latest_amcl_x) and not math.isnan(gt_x):
            error_dist = math.sqrt((self.latest_amcl_x - gt_x)**2 + (self.latest_amcl_y - gt_y)**2)
            
        log_line = f"{sim_time:.3f},{self.latest_scan_min:.3f},{self.latest_amcl_x:.3f},{self.latest_amcl_y:.3f},{self.latest_amcl_cov_x:.5f},{self.latest_amcl_cov_y:.5f},{gt_x:.3f},{gt_y:.3f},{gt_yaw:.3f},{error_dist:.3f}\n"
        self.log_file.write(log_line)
        self.log_file.flush()
        
        # Also print to stdout for easy reading
        print(f"[Time:{sim_time:.1f}] scan_min:{self.latest_scan_min:.3f} | amcl:({self.latest_amcl_x:.2f},{self.latest_amcl_y:.2f}) cov:({self.latest_amcl_cov_x:.3f},{self.latest_amcl_cov_y:.3f}) | gt:({gt_x:.2f},{gt_y:.2f},{gt_yaw:.2f}) | err:{error_dist:.3f}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = EvidenceCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.log_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
