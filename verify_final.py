#!/usr/bin/env python3
import sys
import os
import time
import subprocess
import re
import math
import json

# Ensure ROS 2 path
ros_python_path = '/opt/ros/jazzy/lib/python3.12/site-packages'
if ros_python_path not in sys.path:
    sys.path.insert(0, ros_python_path)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError as e:
    print(f"Failed to import ROS 2: {e}")
    sys.exit(1)

from evaluate_mecanum import start_simulation, stop_simulation, MecanumEvaluator, wait_for_simulation_ready

def get_physical_pose(retries=5, delay=1.0):
    for i in range(retries):
        try:
            res = subprocess.run(
                ["gz", "model", "-m", "visual_amr", "-p"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=15.0  # Extend timeout to 15 seconds
            )
            if res.returncode == 0:
                lines = res.stdout.strip().split('\n')
                pose_lines = []
                for line in reversed(lines):
                    m = re.findall(r'\[\s*(-?\d+\.?\d*(?:e-?\d+)?)\s+(-?\d+\.?\d*(?:e-?\d+)?)\s+(-?\d+\.?\d*(?:e-?\d+)?)\s*\]', line)
                    if m:
                        pose_lines.append([float(x) for x in m[0]])
                        if len(pose_lines) == 2:
                            break
                
                if len(pose_lines) == 2:
                    xyz = pose_lines[1]
                    rpy = pose_lines[0]
                    return (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])
            else:
                print(f"⚠️ [Attempt {i+1}/{retries}] gz model returned exit code {res.returncode}. Stderr: {res.stderr.strip()}")
        except subprocess.TimeoutExpired:
            print(f"⚠️ [Attempt {i+1}/{retries}] gz model query timed out (15s limit).")
        except Exception as e:
            print(f"⚠️ [Attempt {i+1}/{retries}] Unexpected error during gz query: {e}")
        time.sleep(delay)
    return None

def wait_for_simulation_ready_local():
    start_time = time.time()
    while time.time() - start_time < 30.0:
        pose = get_physical_pose(retries=1, delay=0.1)
        if pose is not None:
            return pose
        time.sleep(1.0)
    return None

def main():
    print("🚀 Running robust final verification simulation...")
    proc, env = start_simulation()
    
    try:
        os.environ["ROS_DOMAIN_ID"] = "45"
        rclpy.init()
        
        print("⏳ Waiting for simulator to initialize pose...")
        p0 = wait_for_simulation_ready_local()
        if p0 is None:
            print("❌ Failed to query initial pose from simulator.")
            return
            
        print(f"Initial physical pose: X={p0[0]:.4f}, Y={p0[1]:.4f}, Z={p0[2]:.4f}, R={p0[3]:.4f}, P={p0[4]:.4f}, Yw={p0[5]:.4f}")
        
        evaluator = MecanumEvaluator()
        time.sleep(1.0)
        
        send_duration = 2.0
        stop_msg = Twist()
        
        # 1. Forward Test (linear.x = 0.5)
        print("➡️ Command: Forward (linear.x = 0.5)")
        msg = Twist()
        msg.linear.x = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        # Stop and wait
        t_stop = time.time()
        while time.time() - t_stop < 1.5:
            evaluator.pub.publish(stop_msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        p1 = get_physical_pose()
        if p1 is None:
            print("❌ Failed to get pose after Forward Test.")
            evaluator.destroy_node()
            return
        print(f"Pose after Forward: X={p1[0]:.4f}, Y={p1[1]:.4f}, Z={p1[2]:.4f}, R={p1[3]:.4f}, P={p1[4]:.4f}, Yw={p1[5]:.4f}")
            
        # 2. Slide Test (linear.y = -0.5)
        print("➡️ Command: Slide Right (linear.y = -0.5)")
        msg = Twist()
        msg.linear.y = -0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        # Stop and wait
        t_stop = time.time()
        while time.time() - t_stop < 1.5:
            evaluator.pub.publish(stop_msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        p2 = get_physical_pose()
        if p2 is None:
            print("❌ Failed to get pose after Slide Test.")
            evaluator.destroy_node()
            return
        print(f"Pose after Slide: X={p2[0]:.4f}, Y={p2[1]:.4f}, Z={p2[2]:.4f}, R={p2[3]:.4f}, P={p2[4]:.4f}, Yw={p2[5]:.4f}")
            
        # 3. Turn Test (angular.z = 0.5)
        print("➡️ Command: Turn Left (angular.z = 0.5)")
        msg = Twist()
        msg.angular.z = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        # Stop and wait
        t_stop = time.time()
        while time.time() - t_stop < 1.5:
            evaluator.pub.publish(stop_msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            
        p3 = get_physical_pose()
        if p3 is None:
            print("❌ Failed to get pose after Turn Test.")
            evaluator.destroy_node()
            return
        print(f"Pose after Turn: X={p3[0]:.4f}, Y={p3[1]:.4f}, Z={p3[2]:.4f}, R={p3[3]:.4f}, P={p3[4]:.4f}, Yw={p3[5]:.4f}")
        
        evaluator.destroy_node()
        
        if p1 and p2 and p3:
            dyaw_fwd = math.atan2(math.sin(p1[5] - p0[5]), math.cos(p1[5] - p0[5]))
            dyaw_slide = math.atan2(math.sin(p2[5] - p1[5]), math.cos(p2[5] - p1[5]))
            dyaw_turn = math.atan2(math.sin(p3[5] - p2[5]), math.cos(p3[5] - p2[5]))
            
            print("\n==================================================")
            print("🏁 FINAL VERIFICATION METRICS (Physical Pose)")
            print("==================================================")
            print(f"Forward Command (linear.x = 0.5 for 2.0s):")
            print(f"  - X Change (dx)   : {p1[0] - p0[0]:.6f} m (Command axis)")
            print(f"  - Y Change (dy)   : {p1[1] - p0[1]:.6f} m (Drift, Target: 0)")
            print(f"  - Yaw Change(dyaw): {dyaw_fwd:.6f} rad (Drift, Target: 0)")
            print(f"Slide Command (linear.y = -0.5 for 2.0s):")
            print(f"  - X Change (dx)   : {p2[0] - p1[0]:.6f} m (Drift, Target: 0)")
            print(f"  - Y Change (dy)   : {p2[1] - p1[1]:.6f} m (Command axis)")
            print(f"  - Yaw Change(dyaw): {dyaw_slide:.6f} rad (Drift, Target: 0)")
            print(f"Turn Command (angular.z = 0.5 for 2.0s):")
            print(f"  - X Change (dx)   : {p3[0] - p2[0]:.6f} m (Drift, Target: 0)")
            print(f"  - Y Change (dy)   : {p3[1] - p2[1]:.6f} m (Drift, Target: 0)")
            print(f"  - Yaw Change(dyaw): {dyaw_turn:.6f} rad (Command axis)")
            print("==================================================")
        else:
            print("❌ Failed to query pose coordinates during test sequence.")
            
    except Exception as e:
        print(f"❌ Execution error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        stop_simulation(proc)
        print("Final verification completed.")

if __name__ == '__main__':
    main()
