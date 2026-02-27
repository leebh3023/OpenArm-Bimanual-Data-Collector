#!/usr/bin/env python3
import h5py
import numpy as np
import sys
import os

def analyze_dataset(name, data):
    print(f"\n--- Analyzing {name} ---")
    print(f"Shape: {data.shape}")
    
    if len(data) == 0:
        print("Error: Dataset is empty.")
        return

    # Joint analysis
    if data.ndim == 2:
        num_joints = data.shape[1]
        for i in range(num_joints):
            joint_data = data[:, i]
            j_min = np.min(joint_data)
            j_max = np.max(joint_data)
            j_mean = np.mean(joint_data)
            j_std = np.std(joint_data)
            
            status = "OK"
            if j_std < 1e-6:
                status = "FROZEN (Static Value)"
            elif abs(j_min) > 2*np.pi or abs(j_max) > 2*np.pi:
                status = "POTENTIAL SCALE ISSUE (Out of [-2pi, 2pi])"
            
            print(f"  Joint {i+1}: min={j_min:8.4f}, max={j_max:8.4f}, mean={j_mean:8.4f}, std={j_std:8.4f} -> {status}")
    else:
        print(f"  Stats: min={np.min(data):.4f}, max={np.max(data):.4f}, mean={np.mean(data):.4f}, std={np.std(data):.4f}")

def check_timestamps(ts):
    print("\n--- Timestamp Analysis ---")
    if len(ts) < 2:
        print("Not enough timestamps for analysis.")
        return
    
    diffs = np.diff(ts)
    hz = 1.0 / np.mean(diffs)
    print(f"  Average frequency: {hz:.2f} Hz")
    print(f"  Min/Max interval: {np.min(diffs):.4f}s / {np.max(diffs):.4f}s")
    print(f"  Interval variance: {np.var(diffs):.8f}")
    
    if np.max(diffs) > 2.0 * np.mean(diffs):
        print("  WARNING: Significant jitter or dropped frames detected.")

def analyze_h5(file_path):
    if not os.path.exists(file_path):
        print(f"File not found: {file_path}")
        return

    try:
        with h5py.File(file_path, 'r') as f:
            obs = f['observations']
            
            if 'left_q' in obs:
                analyze_dataset("Left Joints (left_q)", obs['left_q'][:])
            
            if 'right_q' in obs:
                analyze_dataset("Right Joints (right_q)", obs['right_q'][:])
                
            if 'timestamp' in obs:
                check_timestamps(obs['timestamp'][:])
                
            print("\n--- Image Data Check ---")
            if 'images' in obs:
                img_grp = obs['images']
                for cam in img_grp.keys():
                    shape = img_grp[cam].shape
                    print(f"  Camera '{cam}': Shape {shape}")
                    if shape[0] == 0:
                        print(f"    WARNING: Camera '{cam}' has 0 frames recorded.")
            else:
                print("  No image group found.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_h5_data.py <path_to_h5>")
    else:
        analyze_h5(sys.argv[1])
