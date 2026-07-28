#!/usr/bin/env python3
import sys
import os
import time
import subprocess
import math
import json
import itertools

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

def test_config(fl, fr, rl, rr):
    print(f"Testing: FL={fl}, FR={fr}, RL={rl}, RR={rr} ...")
    update_xacro(fl, fr, rl, rr)
    if not build_workspace():
        print("❌ Build failed!")
        return None
        
    proc, env = start_simulation()
    results = None
    try:
        os.environ["ROS_DOMAIN_ID"] = "45"
        if not rclpy.ok():
            rclpy.init()
            
        # Wait for simulation to be ready and get initial pose
        p0 = wait_for_simulation_ready()
        if p0 is None:
            print("❌ Simulation ready timeout.")
            stop_simulation(proc)
            return None
            
        evaluator = MecanumEvaluator()
        # Give some extra time for controllers to spin up
        time.sleep(2.0)
        
        send_duration = 2.0
        stop_msg = Twist()
        
        # 1. Forward Test (linear.x = 0.5)
        msg = Twist()
        msg.linear.x = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
        evaluator.pub.publish(stop_msg)
        time.sleep(1.5)
        p1 = get_physical_pose()
        
        # 2. Slide Test (linear.y = -0.5)
        msg = Twist()
        msg.linear.y = -0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
        evaluator.pub.publish(stop_msg)
        time.sleep(1.5)
        p2 = get_physical_pose()
        
        # 3. Turn Test (angular.z = 0.5)
        msg = Twist()
        msg.angular.z = 0.5
        t_start = time.time()
        while time.time() - t_start < send_duration:
            evaluator.pub.publish(msg)
            rclpy.spin_once(evaluator, timeout_sec=0.1)
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
        print(f"Error during test: {e}")
    finally:
        stop_simulation(proc)
        
    return results

def main():
    options = ["1 1 0", "1 -1 0"]
    # All 16 permutations
    configs = list(itertools.product(options, repeat=4))
    
    print(f"Total configurations to test: {len(configs)}")
    
    log_file = open('/home/pakku/grid_search.log', 'w')
    log_file.write("fl,fr,rl,rr,fwd_dx,fwd_dy,fwd_dyaw,slide_dx,slide_dy,slide_dyaw,turn_dx,turn_dy,turn_dyaw\n")
    log_file.flush()
    
    for idx, (fl, fr, rl, rr) in enumerate(configs):
        print(f"\n--- Running config {idx+1}/{len(configs)} ---")
        # Clean up any leftover processes before starting
        subprocess.run("pkill -9 -f gz-sim; pkill -9 -f parameter_bridge; pkill -9 -f spawner; true", shell=True)
        time.sleep(1.0)
        
        res = test_config(fl, fr, rl, rr)
        if res:
            fwd = res['forward']
            sld = res['slide']
            trn = res['turn']
            print(f"  Forward : dx={fwd['dx']:.4f}, dy={fwd['dy']:.4f}, dyaw={fwd['dyaw']:.4f}")
            print(f"  Slide   : dx={sld['dx']:.4f}, dy={sld['dy']:.4f}, dyaw={sld['dyaw']:.4f}")
            print(f"  Turn    : dx={trn['dx']:.4f}, dy={trn['dy']:.4f}, dyaw={trn['dyaw']:.4f}")
            
            line = f"{fl},{fr},{rl},{rr},{fwd['dx']:.6f},{fwd['dy']:.6f},{fwd['dyaw']:.6f},{sld['dx']:.6f},{sld['dy']:.6f},{sld['dyaw']:.6f},{trn['dx']:.6f},{trn['dy']:.6f},{trn['dyaw']:.6f}\n"
            log_file.write(line)
            log_file.flush()
        else:
            print("  ❌ Test failed.")
            
    log_file.close()
    print("\nGrid search completed. Results written to /home/pakku/grid_search.log")

if __name__ == '__main__':
    main()
