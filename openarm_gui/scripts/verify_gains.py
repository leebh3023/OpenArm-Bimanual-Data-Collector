import sys
import os
import numpy as np

# Adjust path to import OpenArmCANController
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))
from core.can_controller import OpenArmCANController

def test_gain_loading():
    print("--- Testing Gain Loading ---")
    controller = OpenArmCANController()
    
    active_kp = controller.control_gains['active']['kp']
    active_kd = controller.control_gains['active']['kd']
    
    print(f"Loaded KP: {active_kp}")
    print(f"Loaded KD: {active_kd}")
    
    # Mocking self.buses and other needed parts to test _send_mit_commands logic indirectly
    # Or just check if the logic handles the lists correctly.
    
    print("\n--- Simulating Joint Gain Assignment ---")
    for i in range(8):
        motor_id = i + 1
        if motor_id == 8:
            kp = controller.control_gains['gripper']['kp']
            kd = controller.control_gains['gripper']['kd']
        else:
            kp_val = active_kp
            kd_val = active_kd
            
            if isinstance(kp_val, (list, np.ndarray)) and len(kp_val) >= 7:
                kp = kp_val[i]
            else:
                kp = kp_val
                
            if isinstance(kd_val, (list, np.ndarray)) and len(kd_val) >= 7:
                kd = kd_val[i]
            else:
                kd = kd_val
        
        print(f"Motor ID {motor_id}: KP={kp}, KD={kd}")

if __name__ == "__main__":
    test_gain_loading()
