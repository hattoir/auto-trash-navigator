#!/usr/bin/env python3
import sys
import os
import re
import time
import subprocess
import math
import signal
import argparse
import json

# Ensure ROS 2 path is in sys.path
ros_python_path = '/opt/ros/jazzy/lib/python3.12/site-packages'
if ros_python_path not in sys.path:
    sys.path.insert(0, ros_python_path)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
except ImportError as e:
    print(json.dumps({"success": False, "error": f"Failed to import ROS 2 library: {e}"}))
    sys.exit(1)

XACRO_PATH = '/home/pakku/auto-trash-navigator/src/amr_description/urdf/base_omni_4wheel.xacro'

def update_xacro(fl, fr, rl, rr):
    with open(XACRO_PATH, 'r') as f:
        content = f.read()

    content = re.sub(r'(prefix="front_left"\s+x_reflect="1"\s+y_reflect="1"\s+fdir=")[^"]+(")', rf'\g<1>{fl}\g<2>', content)
    content = re.sub(r'(prefix="front_right"\s+x_reflect="1"\s+y_reflect="-1"\s+fdir=")[^"]+(")', rf'\g<1>{fr}\g<2>', content)
    content = re.sub(r'(prefix="rear_left"\s+x_reflect="-1"\s+y_reflect="1"\s+fdir=")[^"]+(")', rf'\g<1>{rl}\g<2>', content)
    content = re.sub(r'(prefix="rear_right"\s+x_reflect="-1"\s+y_reflect="-1"\s+fdir=")[^"]+(")', rf'\g<1>{rr}\g<2>', content)

    with open(XACRO_PATH, 'w') as f:
        f.write(content)

def build_workspace():
    res = subprocess.run(
        ["colcon", "build", "--packages-select", "amr_description"],
        cwd="/home/pakku/auto-trash-navigator",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    return res.returncode == 0

def start_simulation():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "45"
    
    cmd = [
        "ros2", "launch", "amr_bringup", "gazebo.launch.py",
        "gz_args:=-r -s empty.sdf"
    ]
    
    proc = subprocess.Popen(
        cmd,
        cwd="/home/pakku/auto-trash-navigator",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
        env=env
    )
    return proc, env

def stop_simulation(proc):
    if proc:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait()
            except Exception:
                pass
    time.sleep(1.0)

_last_xyz = None

def get_physical_pose(retries=5, delay=0.5):
    global _last_xyz
    for i in range(retries):
        try:
            res = subprocess.run(
                ["gz", "model", "-m", "visual_amr", "-p"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10.0
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
                    
                    # If coordinates match the last state exactly, the simulator transport is lagging.
                    # Force a sleep and retry.
                    if _last_xyz is not None and abs(xyz[0] - _last_xyz[0]) < 1e-5 and abs(xyz[1] - _last_xyz[1]) < 1e-5:
                        time.sleep(0.3)
                        continue
                    
                    _last_xyz = xyz
                    return (xyz[0], xyz[1], rpy[2])
        except Exception:
            pass
        time.sleep(delay)
    return _last_xyz

def wait_for_simulation_ready():
    start_time = time.time()
    while time.time() - start_time < 30.0:
        pose = get_physical_pose(retries=1, delay=0.1)
        if pose is not None:
            return pose
        time.sleep(1.0)
    return None

class MecanumEvaluator(Node):
    def __init__(self):
        super().__init__('mecanum_evaluator')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

def main():
    global _last_xyz
    parser = argparse.ArgumentParser()
    parser.add_argument('--fl', required=True)
    parser.add_argument('--fr', required=True)
    parser.add_argument('--rl', required=True)
    parser.add_argument('--rr', required=True)
    args = parser.parse_args()
    
    update_xacro(args.fl, args.fr, args.rl, args.rr)
    
    if not build_workspace():
        print(json.dumps({"success": False, "error": "Build failed"}))
        sys.exit(1)
        
    proc, test_env = start_simulation()
    
    test_results = None
    err_reason = None
    try:
        os.environ["ROS_DOMAIN_ID"] = "45"
        rclpy.init()
        
        # Settle simulator
        time.sleep(3.0)
        
        p0 = wait_for_simulation_ready()
        if p0 is None:
            err_reason = "Simulation ready timeout: cannot query physical pose"
        else:
            _last_xyz = [p0[0], p0[1], p0[2]]
            evaluator = MecanumEvaluator()
            time.sleep(1.0)
            
            send_duration = 1.5
            brake_duration = 2.0  # Let it fully stop
            stop_msg = Twist()
            
            # 1. Forward Test (linear.x = 0.5)
            msg = Twist()
            msg.linear.x = 0.5
            t_start = time.time()
            while time.time() - t_start < send_duration:
                evaluator.pub.publish(msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
                
            t_stop = time.time()
            while time.time() - t_stop < brake_duration:
                evaluator.pub.publish(stop_msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
            p1 = get_physical_pose()
            
            # 2. Slide Test (linear.y = -0.5)
            msg = Twist()
            msg.linear.y = -0.5
            t_start = time.time()
            while time.time() - t_start < send_duration:
                evaluator.pub.publish(msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
                
            t_stop = time.time()
            while time.time() - t_stop < brake_duration:
                evaluator.pub.publish(stop_msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
            p2 = get_physical_pose()
            
            # 3. Turn Test (angular.z = 0.5)
            msg = Twist()
            msg.angular.z = 0.5
            t_start = time.time()
            while time.time() - t_start < send_duration:
                evaluator.pub.publish(msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
                
            t_stop = time.time()
            while time.time() - t_stop < brake_duration:
                evaluator.pub.publish(stop_msg)
                rclpy.spin_once(evaluator, timeout_sec=0.1)
            p3 = get_physical_pose()
            
            evaluator.destroy_node()
            
            if p1 and p2 and p3:
                # Convert changes to robot local frame
                dx_w = p1[0] - p0[0]
                dy_w = p1[1] - p0[1]
                theta = p0[2]
                dx_l_fwd = dx_w * math.cos(theta) + dy_w * math.sin(theta)
                dy_l_fwd = -dx_w * math.sin(theta) + dy_w * math.cos(theta)
                dyaw_fwd = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
                
                dx_w_s = p2[0] - p1[0]
                dy_w_s = p2[1] - p1[1]
                theta_s = p1[2]
                dx_l_slide = dx_w_s * math.cos(theta_s) + dy_w_s * math.sin(theta_s)
                dy_l_slide = -dx_w_s * math.sin(theta_s) + dy_w_s * math.cos(theta_s)
                dyaw_slide = math.atan2(math.sin(p2[2] - p1[2]), math.cos(p2[2] - p1[2]))
                
                dx_w_t = p3[0] - p2[0]
                dy_w_t = p3[1] - p2[1]
                theta_t = p2[2]
                dx_l_turn = dx_w_t * math.cos(theta_t) + dy_w_t * math.sin(theta_t)
                dy_l_turn = -dx_w_t * math.sin(theta_t) + dy_w_t * math.cos(theta_t)
                dyaw_turn = math.atan2(math.sin(p3[2] - p2[2]), math.cos(p3[2] - p2[2]))
                
                test_results = {
                    'forward': {'dx': dx_l_fwd, 'dy': dy_l_fwd, 'dyaw': dyaw_fwd},
                    'slide': {'dx': dx_l_slide, 'dy': dy_l_slide, 'dyaw': dyaw_slide},
                    'turn': {'dx': dx_l_turn, 'dy': dy_l_turn, 'dyaw': dyaw_turn}
                }
            else:
                err_reason = "Failed to capture physical poses during tests"
    except Exception as e:
        err_reason = f"ROS run error: {e}"
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        stop_simulation(proc)
        
    if test_results:
        print(json.dumps({"success": True, "results": test_results}))
    else:
        print(json.dumps({"success": False, "error": err_reason or "Unknown evaluation error"}))

if __name__ == '__main__':
    main()
