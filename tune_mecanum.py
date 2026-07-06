#!/usr/bin/env python3
import subprocess
import itertools
import json
import os
import sys

XACRO_PATH = '/home/pakku/auto-trash-navigator/src/amr_description/urdf/base_omni_4wheel.xacro'

def main():
    # We will test combination of "1 1 0", "1 -1 0", "-1 1 0", "-1 -1 0" to comprehensively cover
    # local rotations and flips on all four wheels.
    choices = ["1 1 0", "1 -1 0", "-1 1 0", "-1 -1 0"]
    
    # To reduce computation and focus on symmetric configurations, we can define:
    # FL/RL as baseline or search combinations.
    # In general mecanum configurations, FL and RR are matching pairs, FR and RL are matching pairs.
    # We will generate all patterns. Since 4^4 = 256 configurations, this might take too long.
    # Instead, we can restrict the search space by assuming:
    # 1. FL and RR are diagonal pairs, so they must be equal or opposite.
    # 2. FR and RL are diagonal pairs, so they must be equal or opposite.
    # Let's run a smart search. To be absolutely sure, we can iterate all 16 patterns of the standard
    # 2-axis options (choices_reduced = ["1 1 0", "1 -1 0", "-1 1 0", "-1 -1 0"]) for FL/RL,
    # and compute FR/RR based on them or just search a subset.
    #
    # Actually, let's search 16 candidate configurations of:
    # FL: [1 1 0, 1 -1 0, -1 1 0, -1 -1 0]
    # FR: [1 1 0, 1 -1 0, -1 1 0, -1 -1 0]
    # RL: [1 1 0, 1 -1 0, -1 1 0, -1 -1 0]
    # RR: [1 1 0, 1 -1 0, -1 1 0, -1 -1 0]
    # Let's define a list of 24 representative patterns to keep the runtime reasonable (~2-3 minutes).
    # These include standard X-shapes, local Z-rotation compensated shapes, and previous baselines.
    
    patterns = [
        # 1. Baseline non-symmetric
        ("1 1 0", "1 1 0", "1 -1 0", "1 1 0"),
        
        # 2. Standard X-shapes (body coordinates expressed_in = base_footprint)
        ("1 1 0", "1 -1 0", "1 -1 0", "1 1 0"),
        ("1 -1 0", "1 1 0", "1 1 0", "1 -1 0"),
        
        # 3. Local Z-rotation compensated shapes (assuming local coordinate interpretation)
        ("1 1 0", "-1 1 0", "1 -1 0", "-1 -1 0"), # Local Pattern A
        ("1 -1 0", "-1 -1 0", "1 1 0", "-1 1 0"), # Local Pattern B
        ("-1 -1 0", "1 -1 0", "-1 1 0", "1 1 0"),
        ("-1 1 0", "1 1 0", "-1 -1 0", "1 -1 0"),
        
        # 4. Variations with alternate sign patterns
        ("1 1 0", "1 -1 0", "1 1 0", "1 -1 0"),
        ("1 -1 0", "1 1 0", "1 -1 0", "1 1 0"),
        ("-1 1 0", "-1 1 0", "1 -1 0", "1 -1 0"),
        ("1 -1 0", "1 -1 0", "-1 1 0", "-1 1 0"),
        
        # 5. Flip combinations (swapping Y signs)
        ("1 1 0", "-1 -1 0", "1 -1 0", "-1 1 0"),
        ("1 -1 0", "-1 1 0", "1 1 0", "-1 -1 0"),
        
        # 6. Mirror patterns
        ("-1 1 0", "1 -1 0", "-1 1 0", "1 -1 0"),
        ("1 1 0", "-1 1 0", "-1 1 0", "1 1 0"),
        ("-1 1 0", "1 1 0", "1 1 0", "-1 1 0")
    ]
    
    print(f"Total patterns to evaluate: {len(patterns)}")
    
    best_pattern = None
    min_penalty = float('inf')
    best_results = None
    
    with open(XACRO_PATH, 'r') as f:
        original_xacro = f.read()
        
    results_log = []
    
    try:
        for idx, (fl, fr, rl, rr) in enumerate(patterns):
            print(f"\n==================================================")
            print(f"[{idx+1}/{len(patterns)}] Evaluating configuration:")
            print(f"  FL: {fl} | FR: {fr} | RL: {rl} | RR: {rr}")
            print(f"==================================================")
            
            cmd = [
                "python3", "evaluate_mecanum.py",
                "--fl", fl, "--fr", fr, "--rl", rl, "--rr", rr
            ]
            
            res = subprocess.run(
                cmd,
                cwd="/home/pakku/auto-trash-navigator",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            output_json = None
            for line in res.stdout.strip().split('\n'):
                try:
                    data = json.loads(line)
                    if "success" in data:
                        output_json = data
                        break
                except json.JSONDecodeError:
                    continue
            
            if output_json is None or not output_json.get("success"):
                err_msg = output_json.get("error") if output_json else "Unknown execution error"
                print(f"❌ Failed: {err_msg}")
                if res.stderr:
                    print(f"Stderr: {res.stderr}")
                # Clean up zombies
                subprocess.run("pkill -f gz-sim; pkill -f parameter_bridge; pkill -f spawner; true", shell=True)
                continue
                
            test_results = output_json["results"]
            print(f"  Forward (Local): dx={test_results['forward']['dx']:.4f}, dy={test_results['forward']['dy']:.4f}, dyaw={test_results['forward']['dyaw']:.4f}")
            print(f"  Slide   (Local): dx={test_results['slide']['dx']:.4f}, dy={test_results['slide']['dy']:.4f}, dyaw={test_results['slide']['dyaw']:.4f}")
            print(f"  Turn    (Local): dx={test_results['turn']['dx']:.4f}, dy={test_results['turn']['dy']:.4f}, dyaw={test_results['turn']['dyaw']:.4f}")
            
            # Polarity validation checks (ensure robot moves in the correct command direction)
            fwd_ok = test_results['forward']['dx'] > 0.05
            slide_ok = test_results['slide']['dy'] < -0.05 # Commands negative Y sliding (Right slide)
            turn_ok = test_results['turn']['dyaw'] > 0.05 # Commands positive Z angular rotation (Left turn)
            
            if not (fwd_ok and slide_ok and turn_ok):
                print("⚠️ Invalid motion directions or reversed polarities. Assigning maximum penalty.")
                penalty = 9999.0
            else:
                # Drift penalty terms (minimize offsets on zero-velocity axes)
                # Forward drift: dy and dyaw should be 0
                fwd_err = abs(test_results['forward']['dy'])*2.0 + abs(test_results['forward']['dyaw']) * 10.0
                # Slide drift: dx and dyaw should be 0 (Highly weight slide yaw stability)
                slide_err = abs(test_results['slide']['dx'])*2.0 + abs(test_results['slide']['dyaw']) * 20.0
                # Turn drift: dx and dy should be 0
                turn_err = abs(test_results['turn']['dx'])*2.0 + abs(test_results['turn']['dy'])*2.0
                
                penalty = fwd_err + slide_err + turn_err
                print(f"  -> Calculated Penalty Score: {penalty:.6f}")
                
            results_log.append({
                'pattern': (fl, fr, rl, rr),
                'results': test_results,
                'penalty': penalty
            })
            
            if penalty < min_penalty:
                min_penalty = penalty
                best_pattern = (fl, fr, rl, rr)
                best_results = test_results
                print(f"✨ New Best Configuration Found! Penalty: {min_penalty:.6f}")
                
    finally:
        if best_pattern:
            fl, fr, rl, rr = best_pattern
            print(f"\n==================================================")
            print(f"🏁 EVALUATION COMPLETE")
            print(f"Best configuration:")
            print(f"  FL: {fl} | FR: {fr} | RL: {rl} | RR: {rr}")
            print(f"  Min Penalty Score: {min_penalty:.6f}")
            print(f"  Forward (Local): dx={best_results['forward']['dx']:.4f}, dy={best_results['forward']['dy']:.4f}, dyaw={best_results['forward']['dyaw']:.4f}")
            print(f"  Slide   (Local): dx={best_results['slide']['dx']:.4f}, dy={best_results['slide']['dy']:.4f}, dyaw={best_results['slide']['dyaw']:.4f}")
            print(f"  Turn    (Local): dx={best_results['turn']['dx']:.4f}, dy={best_results['turn']['dy']:.4f}, dyaw={best_results['turn']['dyaw']:.4f}")
            print(f"==================================================")
            
            sys.path.insert(0, '/home/pakku/auto-trash-navigator')
            from evaluate_mecanum import update_xacro, build_workspace
            update_xacro(fl, fr, rl, rr)
            build_workspace()
            print("Configured base_omni_4wheel.xacro with the best physical parameters and rebuilt.")
        else:
            with open(XACRO_PATH, 'w') as f:
                f.write(original_xacro)
            subprocess.run(["colcon", "build", "--packages-select", "amr_description"], cwd="/home/pakku/auto-trash-navigator")
            print("No valid configuration found. Restored original URDF and rebuilt.")

if __name__ == '__main__':
    main()
