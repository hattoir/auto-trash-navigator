#!/usr/bin/env python3
"""
ROS 2 Jazzy Autonomous Driving Node.

This node subscribes to camera images, preprocesses them, passes them through a trained
PyTorch regression CNN (DrivingCNN) to predict linear and angular velocities, and publishes
these commands to /cmd_vel to steer the Gazebo vehicle.
"""

import sys
import os
import argparse
import cv2
import numpy as np

import torch
import torch.nn as nn
import torchvision.transforms as transforms

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry

# Define the exact DrivingCNN architecture used during training
class DrivingCNN(nn.Module):
    def __init__(self):
        super(DrivingCNN, self).__init__()
        
        self.features = nn.Sequential(
            nn.Conv2d(3, 24, kernel_size=5, stride=2),
            nn.BatchNorm2d(24),
            nn.ReLU(),

            nn.Conv2d(24, 36, kernel_size=5, stride=2),
            nn.BatchNorm2d(36),
            nn.ReLU(),

            nn.Conv2d(36, 48, kernel_size=5, stride=2),
            nn.BatchNorm2d(48),
            nn.ReLU(),

            nn.Conv2d(48, 64, kernel_size=3, stride=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),

            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
        )
        
        self.pool = nn.AdaptiveAvgPool2d((4, 4))
        
        self.fc = nn.Sequential(
            nn.Linear(64 * 4 * 4, 100),
            nn.ReLU(),
            nn.Linear(100, 50),
            nn.ReLU(),
            nn.Linear(50, 10),
            nn.ReLU(),
            nn.Linear(10, 2)  # Output: [predicted_linear_x, predicted_angular_z]
        )

    def forward(self, x):
        x = self.features(x)
        x = self.pool(x)
        x = torch.flatten(x, start_dim=1)
        x = self.fc(x)
        return x

class AutonomousDriver(Node):
    def __init__(self, model_path):
        super().__init__('autonomous_driver')
        
        self.get_logger().info("[autonomous_driver]: Initializing Autonomous Driving Control Node...")
        
        # 1. Device selection
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"[autonomous_driver]: Using device: {self.device.type.upper()}")
        
        # 2. Load PyTorch model
        self.model = DrivingCNN().to(self.device)
        try:
            # Load weights mapping to current device
            self.model.load_state_dict(torch.load(model_path, map_location=self.device))
            self.model.eval()
            self.get_logger().info(f"[autonomous_driver]: Successfully loaded PyTorch model from: {model_path}")
        except Exception as e:
            self.get_logger().error(f"[autonomous_driver]: Failed to load PyTorch model weights: {str(e)}")
            sys.exit(1)
            
        # 3. Setup Image transformations matching the training script
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((120, 160)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        self.bridge = CvBridge()

        # Goal line detection and automatic stop configurations
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.goal_reached = False
        self.get_logger().info("[autonomous_driver]: Red starting/goal line detection filter initialized successfully!")
        
        # 4. Setup publishers and subscribers
        # Camera QoS matches the data collector (Best Effort)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            cmd_vel_qos
        )
        
        self.get_logger().info("[autonomous_driver]: Subscribed to '/camera/image_raw' and publishing to '/cmd_vel'. Ready for autonomous driving!")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format (BGR8)
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge conversion error: {str(e)}")
            return

        # --- Red starting line (goal) detection ---
        if not self.goal_reached:
            # Get elapsed time in simulation/clock seconds
            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = current_time - self.start_time
            
            # Check bottom 25% of the frame (where the starting line appears under the nose)
            height, width, _ = cv_img.shape
            bottom_region = cv_img[int(height * 0.75):, :]
            
            # Convert bottom region to HSV
            hsv = cv2.cvtColor(bottom_region, cv2.COLOR_BGR2HSV)
            
            # Bright Red HSV threshold bounds
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            red_mask = mask1 | mask2
            
            red_pixel_count = np.sum(red_mask > 0)
            
            # Check if red line is crossed (after 10s cooldown to allow driving away)
            if elapsed_time > 10.0 and red_pixel_count > 3000:
                self.get_logger().info("[autonomous_driver]: Goal line detected! Vehicle safely stopped.")
                self.goal_reached = True

        # Stop vehicle immediately if goal has been reached
        if self.goal_reached:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            return

        # Convert BGR to RGB (matching training script cvtColor)
        cv_img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        
        # Apply transforms and add batch dimension [1, 3, 120, 160]
        input_tensor = self.transform(cv_img_rgb).unsqueeze(0).to(self.device)
        
        # Model inference
        with torch.no_grad():
            prediction = self.model(input_tensor)
            prediction = prediction.cpu().numpy()[0]
            
        linear_x = float(prediction[0]) * 2.0
        predicted_angular_z = float(prediction[1])
        
        # Apply steering damping factor of 0.85 to prevent hunting (oscillations) while keeping cornering power
        angular_z = predicted_angular_z * 0.85

        # Build Twist command message
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z
        
        # Publish driving commands
        self.cmd_vel_pub.publish(cmd_msg)
        
        self.get_logger().info(
            f"[autonomous_driver]: Predicted linear.x={linear_x:.3f}, angular.z={angular_z:.3f} (raw={predicted_angular_z:.3f})",
            throttle_duration_sec=1.0
        )

def main():
    parser = argparse.ArgumentParser(description="ROS 2 Autonomous Driving Control Node")
    parser.add_argument('--model', type=str, required=True, help="Path to trained PyTorch .pth model file")
    
    # We parse args after rclpy.init to avoid conflict with standard ROS arguments, or parse known args
    args, unknown = parser.parse_known_args()
    
    rclpy.init(args=unknown)
    
    node = AutonomousDriver(args.model)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[autonomous_driver]: Autonomous driving node shutting down cleanly...")
        # Send a stop command on exit
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
