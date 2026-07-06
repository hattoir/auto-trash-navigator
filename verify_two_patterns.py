#!/usr/bin/env python3
import sys
import os
import time
import subprocess
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

from evaluate_mecanum import update_xacro, build_workspace, start_simulation, stop_simulation, get_physical_pose, MecanumEvaluator

def wait_for_simulation_ready():
    start_time = time.time()
    while time.time() - start_time < 30.0:
        pose = get_physical_pose()
        if pose is not None:
            return pose
        time.sleep(1.0)
    return None

def test_pattern(fl, fr, rl, rr):
    print(f"\nEvaluating: FL={fl}, FR={fr}, RL={rl}, RR={rr}")
    update_xacro(fl, fr, rl, rr)
    if not build_workspace():
        print("❌ Build failed!")
        return None
        
    proc, env = start_simulation()
    results = None
    try:
        os.environ["ROS_DOMAIN_ID"] = "45"
        rclpy.init()
        
        p0 = wait_for_simulation_ready()
        if p0 is None:
            print("❌ Simulation ready timeout.")
            return None
            
        evaluator = MecanumEvaluator()
        time.sleep(1.5)
        
        send_duration = 2.0
        stop_msg = Twist()
        
        # 1. Forward Test
        msg = Twist()
        msg.linear.x = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            time.sleep(0.1)
        evaluator.pub.publish(stop_msg)
        time.sleep(1.5)
        p1 = get_physical_pose()
        
        # 2. Slide Test (Y-スライド)
        msg = Twist()
        msg.linear.y = -0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            time.sleep(0.1)
        evaluator.pub.publish(stop_msg)
        time.sleep(1.5)
        p2 = get_physical_pose()
        
        # 3. Turn Test
        msg = Twist()
        msg.angular.z = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            time.sleep(0.1)
        evaluator.pub.publish(stop_msg)
        time.sleep(1.5)
        p3 = get_physical_pose()
        
        evaluator.destroy_node()
        
        if p1 and p2 and p3:
            dyaw_fwd = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
            dyaw_slide = math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2]))
            dyaw_turn = math.atan2(math.sin(p3[2] - p2[2]), math.cos(p3[2] - p2[2]))
            
            results = {
                'forward': {'dx': p1[0] - p0[0], 'dy': p1[1] - p0[1], 'dyaw': dyaw_fwd},
                'slide': {'dx': p2[0] - p1[0], 'dy': p2[1] - p1[1], 'dyaw': dyaw_slide},
                'turn': {'dx': p3[0] - p2[0], 'dy': p3[1] - p2[1], 'dyaw': dyaw_turn}
            }
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        stop_simulation(proc)
        
    return results

def main():
    patterns = {
        'Pattern A (FL/RR: 1 1 0, FR/RL: 1 -1 0)': ("1 1 0", "1 -1 0", "1 -1 0", "1 1 0"),
        'Pattern B (FL/RR: 1 -1 0, FR/RL: 1 1 0)': ("1 -1 0", "1 1 0", "1 1 0", "1 -1 0")
    }
    
    final_reports = {}
    
    for name, config in patterns.items():
        # Clean up any leftover processes before starting
        subprocess.run("pkill -f gz-sim; pkill -f parameter_bridge; pkill -f spawner; true", shell=True)
        time.sleep(1.0)
        
        res = test_pattern(*config)
        if res:
            final_reports[name] = res
            print(f"✅ {name} Results:")
            print(f"  Forward : dx={res['forward']['dx']:.6f}, dy={res['forward']['dy']:.6f}, dyaw={res['forward']['dyaw']:.6f}")
            print(f"  Slide   : dx={res['slide']['dx']:.6f}, dy={res['slide']['dy']:.6f}, dyaw={res['slide']['dyaw']:.6f}")
            print(f"  Turn    : dx={res['turn']['dx']:.6f}, dy={res['turn']['dy']:.6f}, dyaw={res['turn']['dyaw']:.6f}")
        else:
            print(f"❌ {name} Failed to complete.")
            
    print("\n==============================================")
    print("🏆 FINAL COMPARISON REPORT")
    print("==============================================")
    for name, res in final_reports.items():
        # Calculate score (minimize sum of absolute drifts)
        fwd_drift = abs(res['forward']['dy']) + abs(res['forward']['dyaw'])
        slide_drift = abs(res['slide']['dx']) + abs(res['slide']['dyaw'])
        turn_drift = abs(res['turn']['dx']) + abs(res['turn']['dy'])
        total_drift = fwd_drift + slide_drift + turn_drift
        print(f"{name}: Total Drift = {total_drift:.6f}")
    print("==============================================")

if __name__ == '__main__':
    main()
