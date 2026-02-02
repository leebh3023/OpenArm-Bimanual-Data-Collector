import sys
import os
import time
import numpy as np

# Add the source directory to the path so we can import the module
sys.path.append('/home/rvlab/openarm_ws/src/OpenArm-Bimanual-Data-Collector/openarm_gui/src')

from core.can_controller import OpenArmCANController

def test_reading():
    print("Initializing Controller...")
    controller = OpenArmCANController()
    
    print("Connecting...")
    if not controller.connect():
        print("Connection failed!")
        return

    controller.start()
    print("Controller started. Monitoring for 5 seconds...")
    
    start_time = time.time()
    while time.time() - start_time < 5.0:
        state = controller.get_robot_state()
        
        # Print Left Arm Joint 1-3
        l_j = state['left']['joints']
        r_j = state['right']['joints']
        
        # Check if we are receiving data (not just zeros or noise)
        # Note: If motors are off, values might be constant but non-zero.
        
        print(f"\r[Left] J1: {l_j[0]:.4f}, J2: {l_j[1]:.4f}, J3: {l_j[2]:.4f} | [Right] J1: {r_j[0]:.4f}, J2: {r_j[1]:.4f}", end="")
        time.sleep(0.1)
        
    print("\n\nStopping...")
    controller.stop()
    print("Done.")

if __name__ == "__main__":
    test_reading()
