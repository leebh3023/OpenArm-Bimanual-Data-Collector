#!/usr/bin/env python3
import argparse
import h5py
import rerun as rr
import numpy as np
import cv2
import os
import sys

def visualize_openarm_data(h5_path):
    if not os.path.exists(h5_path):
        print(f"Error: File not found: {h5_path}")
        return

    print(f"Loading {h5_path}...")
    
    try:
        f = h5py.File(h5_path, 'r')
    except Exception as e:
        print(f"Error opening HDF5 file: {e}")
        return

    # Initialize Rerun
    rr.init("OpenArm Visualization", spawn=True)

    # Check for valid structure
    if 'observations' not in f:
        print("Error: Invalid HDF5 structure. 'observations' group not found.")
        return

    obs = f['observations']
    
    # Get total frames
    # Assuming 'timestamp' or 'left_q' exists and has length
    if 'timestamp' in obs:
        timestamps = obs['timestamp'][:]
        num_frames = len(timestamps)
    elif 'left_q' in obs:
        num_frames = len(obs['left_q'])
        timestamps = np.arange(num_frames) * (1.0/30.0) # Assume 30Hz if no timestamp
    else:
        print("Error: No timestamp or joint data found.")
        return

    print(f"Visualizing {num_frames} frames...")

    # Iterate through frames
    for i in range(num_frames):
        # Set timeline
        rr.set_time_seconds("stable_time", timestamps[i])
        rr.set_time_sequence("frame_idx", i)

        # 1. Joint States
        if 'left_q' in obs:
            l_q = obs['left_q'][i]
            # l_q is likely 7 or 8 dim
            for j, val in enumerate(l_q):
                label = f"left_joint_{j+1}"
                if j == 7: label = "left_gripper"
                rr.log(f"joints/left/{label}", rr.Scalar(val))

        if 'right_q' in obs:
            r_q = obs['right_q'][i]
            for j, val in enumerate(r_q):
                label = f"right_joint_{j+1}"
                if j == 7: label = "right_gripper"
                rr.log(f"joints/right/{label}", rr.Scalar(val))

        # 2. Images
        if 'images' in obs:
            img_grp = obs['images']
            for cam_name in img_grp.keys():
                # cam_name: top, left, right
                img_ds = img_grp[cam_name]
                if i < len(img_ds):
                    img = img_ds[i]
                    # Check format. OpenCV usually saves BGR if direct from collection, 
                    # but OpenArmMainWindow converts to RGB before saving? 
                    # Let's check main_window.py: Yes, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) is used.
                    # So img is RGB.
                    rr.log(f"cameras/{cam_name}", rr.Image(img))

    print("Visualization complete. Check Rerun viewer.")
    f.close()

def main():
    parser = argparse.ArgumentParser(description="Visualize OpenArm HDF5 data with Rerun")
    parser.add_argument("file", help="Path to .h5 file")
    args = parser.parse_args()

    visualize_openarm_data(args.file)

if __name__ == "__main__":
    main()
