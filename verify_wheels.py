#!/usr/bin/env python3
import sys
import os
import time
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

class WheelVerifier(Node):
    def __init__(self):
        super().__init__('wheel_verifier')
        self.sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_states = {}
        
    def joint_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if 'wheel_joint' in name:
                self.joint_states[name] = pos

def main():
    os.environ["ROS_DOMAIN_ID"] = "45"
    
    # Start Gazebo
    env = os.environ.copy()
    cmd = [
        "ros2", "launch", "amr_bringup", "gazebo.launch.py",
        "headless:=true"
    ]
    log_file = open('/home/pakku/launch_err2.log', 'w')
    proc = subprocess.Popen(
        cmd,
        cwd="/home/pakku/auto-trash-navigator",
        stdout=log_file,
        stderr=log_file,
        preexec_fn=os.setsid,
        env=env
    )
    
    print("⏳ Waiting 10 seconds for simulator and controller manager to start...")
    time.sleep(10.0)
    
    rclpy.init()
    node = WheelVerifier()
    
    # Spin a bit to get initial joint states
    print("Reading initial wheel joint positions...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    init_states = dict(node.joint_states)
    print("Initial wheel joint positions:", init_states)
    
    # Publish cmd_vel
    print("Publishing cmd_vel (linear.y = -0.5) for 3 seconds...")
    twist = Twist()
    twist.linear.y = -0.5
    t_start = time.time()
    while time.time() - t_start < 3.0:
        node.pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.1)
        
    # Spin a bit to get final joint states
    print("Reading final wheel joint positions...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        
    final_states = dict(node.joint_states)
    print("Final wheel joint positions:", final_states)
    
    print("\nDifferences:")
    for name in init_states:
        diff = final_states.get(name, 0.0) - init_states[name]
        print(f"  {name}: {diff:.6f} rad")
        
    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    
    os.killpg(os.getpgid(proc.pid), 9)
    print("Done.")

if __name__ == '__main__':
    main()
