#!/usr/bin/env python3
"""自動テストスクリプト
"""
import subprocess
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

def set_trash_pose(x, y, z=0.012):
    cmd = [
        'gz', 'service',
        '-s', '/world/office_room/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "paper_trash_1", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}'
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return "data: true" in res.stdout

class TestClient(Node):
    def __init__(self):
        super().__init__('test_client', parameter_overrides=[
            rclpy.Parameter('use_sim_time', value=True)
        ])
        self.client = self.create_client(GetPlan, '/pick_trash')
        
    def call_pick_trash(self, x, y, z=0.035):
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Pick-and-place service not available!")
            return False
            
        req = GetPlan.Request()
        req.goal = PoseStamped()
        req.goal.header.frame_id = 'base_footprint'
        req.goal.header.stamp = self.get_clock().now().to_msg()
        req.goal.pose.position.x = float(x)
        req.goal.pose.position.y = float(y)
        req.goal.pose.position.z = float(z)
        req.goal.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Sending pick_trash request for: ({x}, {y}, {z})")
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.done():
            res = future.result()
            # If the resulting plan has poses, it means success
            return len(res.plan.poses) > 0
        return False

def main():
    rclpy.init()
    node = TestClient()
    
    results = {
        'center': [],
        'left': [],
        'right': [],
        'out_of_reach': []
    }
    
    # 1. Center (x=0.28, y=0.0) : 10 times
    print("\n--- Starting Center Tests (10 times) ---")
    for i in range(10):
        print(f"\nCenter Test {i+1}/10...")
        # Reset arm to home (by calling service at home or just waiting)
        # Place trash
        if not set_trash_pose(0.28, 0.0):
            print("Failed to set trash pose in Gazebo!")
            results['center'].append(False)
            continue
        time.sleep(1.0)
        
        success = node.call_pick_trash(0.28, 0.0)
        print(f"Result: {'SUCCESS' if success else 'FAILED'}")
        results['center'].append(success)
        time.sleep(1.0) # Wait for arm to settle
        
    # 2. Left (x=0.28, y=0.15) : 5 times
    print("\n--- Starting Left Tests (5 times) ---")
    for i in range(5):
        print(f"\nLeft Test {i+1}/5...")
        if not set_trash_pose(0.28, 0.15):
            print("Failed to set trash pose in Gazebo!")
            results['left'].append(False)
            continue
        time.sleep(1.0)
        
        success = node.call_pick_trash(0.28, 0.15)
        print(f"Result: {'SUCCESS' if success else 'FAILED'}")
        results['left'].append(success)
        time.sleep(1.0)
        
    # 3. Right (x=0.28, y=-0.15) : 5 times
    print("\n--- Starting Right Tests (5 times) ---")
    for i in range(5):
        print(f"\nRight Test {i+1}/5...")
        if not set_trash_pose(0.28, -0.15):
            print("Failed to set trash pose in Gazebo!")
            results['right'].append(False)
            continue
        time.sleep(1.0)
        
        success = node.call_pick_trash(0.28, -0.15)
        print(f"Result: {'SUCCESS' if success else 'FAILED'}")
        results['right'].append(success)
        time.sleep(1.0)
        
    # 4. Out of reach (x=0.45, y=0.0) : 1 time
    print("\n--- Starting Out of Reach Test (1 time) ---")
    if not set_trash_pose(0.45, 0.0):
        print("Failed to set trash pose in Gazebo!")
        results['out_of_reach'].append(False)
    else:
        time.sleep(1.0)
        success = node.call_pick_trash(0.45, 0.0)
        print(f"Result: {'SUCCESS (Expected FAILURE)' if success else 'FAILED (Expected FAILURE)'}")
        results['out_of_reach'].append(success)
        
    # Print Summary
    print("\n================ TEST SUMMARY ================")
    
    center_success = sum(1 for r in results['center'] if r)
    left_success = sum(1 for r in results['left'] if r)
    right_success = sum(1 for r in results['right'] if r)
    out_of_reach_success = sum(1 for r in results['out_of_reach'] if r)
    
    print(f"Center (x=0.28, y=0.0) : {center_success}/10 success ({(center_success/10)*100:.1f}%)")
    print(f"Left   (x=0.28, y=0.15): {left_success}/5 success ({(left_success/5)*100:.1f}%)")
    print(f"Right  (x=0.28, y=-0.15): {right_success}/5 success ({(right_success/5)*100:.1f}%)")
    print(f"Out of Reach (x=0.45, y=0.0): {out_of_reach_success}/1 success (expected 0/1)")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
