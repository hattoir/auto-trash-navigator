#!/usr/bin/env python3
import sys
import os
import csv
import math
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class ReachabilityMapper(Node):
    def __init__(self):
        super().__init__('reachability_mapper')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service to be available...')
            
        self.param_cli = self.create_client(SetParameters, '/move_group/set_parameters')
        while not self.param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_group/set_parameters service...')
        self.get_logger().info('Services are available. Starting scan...')

    def set_position_only_ik(self, enable: bool):
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable)
        param = Parameter(name='robot_description_kinematics.arm.position_only_ik', value=val)
        req.parameters = [param]
        
        future = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None and res.results[0].successful:
            self.get_logger().info(f"Successfully set position_only_ik to {enable}")
            return True
        else:
            self.get_logger().error(f"Failed to set position_only_ik to {enable}")
            return False

    def check_ik(self, x, y, z, cond_type):
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.ik_link_name = 'link6'
        req.ik_request.avoid_collisions = False
        
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        
        if cond_type == 1:
            # Gripper pointing straight down (rotation of pi around Y axis)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
        elif cond_type == 2:
            # Gripper pointing angled down (pitch=45deg (135deg from Z+), yaw pointing to target)
            yaw = math.atan2(float(y), float(x))
            pitch = 135.0 * math.pi / 180.0
            
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            
            pose.pose.orientation.x = -sp * sy
            pose.pose.orientation.y = sp * cy
            pose.pose.orientation.z = cp * sy
            pose.pose.orientation.w = cp * cy
        elif cond_type == 3:
            # Position only IK (orientation is ignored, but we pass down orientation as seed/target)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 1.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout = Duration(sec=0, nanosec=100000000) # 0.1s
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is not None:
            return res.error_code.val == 1
        return False

def main():
    rclpy.init()
    node = ReachabilityMapper()
    
    z = 0.012
    
    # x ranges from 0.7 down to 0.0 (step 0.05)
    x_coords = []
    curr_x = 0.7
    while curr_x >= -0.001:
        x_coords.append(round(curr_x, 2))
        curr_x -= 0.05
        
    y_coords = []
    curr_y = -0.4
    while curr_y <= 0.401:
        y_coords.append(round(curr_y, 2))
        curr_y += 0.05
        
    conditions = [
        (1, "Condition 1: Gripper Facing Down (pitch=90deg)"),
        (2, "Condition 2: Gripper Angled Down (pitch=45deg, yaw to target)"),
        (3, "Condition 3: Position Only (no orientation constraint)")
    ]
    
    csv_rows = []
    csv_rows.append(['condition', 'x', 'y', 'z', 'success'])
    
    for cond_val, cond_name in conditions:
        print(f"\nScanning for {cond_name}...")
        
        # Set parameter for condition 3
        if cond_val == 3:
            node.set_position_only_ik(True)
        else:
            node.set_position_only_ik(False)
            
        grid_results = {}
        success_xs = []
        success_ys = []
        
        for x in x_coords:
            grid_results[x] = {}
            for y in y_coords:
                success = node.check_ik(x, y, z, cond_val)
                grid_results[x][y] = success
                csv_rows.append([cond_val, x, y, z, 1 if success else 0])
                if success:
                    success_xs.append(x)
                    success_ys.append(y)
                    
        # Print ASCII map
        print(f"\n=== Reachability Map ({cond_name}) ===")
        print("Columns (Y): -0.4m to +0.4m (left to right, step 0.05m)")
        print("Rows (X): 0.7m down to 0.0m (top to bottom, step 0.05m)\n")
        
        # Print Y header indices
        print("      " + " ".join([f"{y: >5}" for y in y_coords]))
        
        for x in x_coords:
            row_str = f"{x: >4.2f} "
            for y in y_coords:
                val = "*" if grid_results[x][y] else "."
                row_str += f"   {val}  "
            print(row_str)
            
        print("\n=================================================================\n")
        
        if success_xs:
            min_x = min(success_xs)
            max_x = max(success_xs)
            max_abs_y = max([abs(y) for y in success_ys])
            print(f"Min successful X: {min_x:.3f} m")
            print(f"Max successful X: {max_x:.3f} m")
            print(f"Max successful absolute Y: {max_abs_y:.3f} m")
        else:
            print("No successful IK solutions found!")
            
    # Reset parameter to False before exiting
    node.set_position_only_ik(False)
    
    # Save to CSV
    csv_path = '/home/pakku/auto-trash-navigator/src/amr_bringup/scripts/reach_map.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(csv_rows)
        
    node.get_logger().info(f"Results saved to {csv_path}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
