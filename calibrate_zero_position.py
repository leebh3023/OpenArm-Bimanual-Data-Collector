#!/usr/bin/env python3
"""
Zero-position calibration using DM_CAN.py
Ported from openarm-can-zero-position-calibration.py
"""

import argparse
import time
import numpy as np
from enum import IntEnum
from DM_CAN import *

# Motor configuration
Motor_CAN0 = {
    1: Motor(DM_Motor_Type.DM8009, 0x01, 0x11),      # J1
    2: Motor(DM_Motor_Type.DM8009, 0x02, 0x12),      # J2
    3: Motor(DM_Motor_Type.DM4340, 0x03, 0x13),  # J3
    4: Motor(DM_Motor_Type.DM4340, 0x04, 0x14),  # J4
    5: Motor(DM_Motor_Type.DM4310, 0x05, 0x15),      # J5
    6: Motor(DM_Motor_Type.DM4310, 0x06, 0x16),      # J6
    7: Motor(DM_Motor_Type.DM4310, 0x07, 0x17),      # J7
    8: Motor(DM_Motor_Type.DM4310, 0x08, 0x18),      # Gripper
}

Motor_CAN1 = {
    1: Motor(DM_Motor_Type.DM8009, 0x01, 0x11),      # J1
    2: Motor(DM_Motor_Type.DM8009, 0x02, 0x12),      # J2
    3: Motor(DM_Motor_Type.DM4340_48V, 0x03, 0x13),  # J3
    4: Motor(DM_Motor_Type.DM4340_48V, 0x04, 0x14),  # J4
    5: Motor(DM_Motor_Type.DM4310, 0x05, 0x15),      # J5
    6: Motor(DM_Motor_Type.DM4310, 0x06, 0x16),      # J6
    7: Motor(DM_Motor_Type.DM4310, 0x07, 0x17),      # J7
    8: Motor(DM_Motor_Type.DM4310, 0x08, 0x18),      # Gripper
}

class JointID(IntEnum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3
    J5 = 4
    J6 = 5
    J7 = 6
    GRIPPER = 7

# Mechanical limits (from original)
mech_lim = {
    JointID.J1: [np.deg2rad(-80),  np.deg2rad(200)],
    JointID.J2: [np.deg2rad(-100), np.deg2rad(100)],
    JointID.J3: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.J4: [np.deg2rad(0),    np.deg2rad(140)],
    JointID.J5: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.J6: [np.deg2rad(-45),  np.deg2rad(45)],
    JointID.J7: [np.deg2rad(-90),  np.deg2rad(90)],
    JointID.GRIPPER: [np.deg2rad(-60), np.deg2rad(0)],
}

# Per-joint sign convention for ideal-zero delta computation (from original)
JOINT_SIGN = {
    JointID.J1: -1.0, JointID.J2: -1.0, JointID.J3: +1.0, JointID.J4: +1.0,
    JointID.J5: +1.0, JointID.J6: +1.0, JointID.J7: +1.0, JointID.GRIPPER: +1.0,
}

# Per-joint kp/kd for initial holding (from original)
HOLD_KP = [300, 300, 150, 150, 40, 40, 30, 10]
HOLD_KD = [2.5, 2.5, 2.5, 2.5, 0.8, 0.8, 0.8, 0.9]

def _hit_thresholds(idx):
    """Return (dq_th, tau_th) based on joint/gripper."""
    if idx == 7:  # gripper is soft
        return 0.3, 0.3
    if idx == 0:  # J1 on arm is stricter
        return 0.0125, 5.0
    return 0.1, 2.0

def bump_to_limit(controller, motor_dict, joint_id: JointID, 
                  step_deg=0.1, kp=30.0, kd=1.2, torque_bias=0.0):
    """Step until mechanical stop. Return traveled delta [rad] (matching original)."""
    idx = int(joint_id)
    motor_idx = idx + 1  # Motor dict is 1-indexed
    motor = motor_dict[motor_idx]
    
    step_rad = np.deg2rad(step_deg)
    tau_cmd = np.copysign(1.0, step_deg) * abs(torque_bias)
    dq_th, tau_th = _hit_thresholds(idx)
    
    q_start = motor.getPosition()
    q_target = q_start
    
    print(f"\n[DEBUG] bump_to_limit {joint_id.name}: start={q_start:.4f} rad, kp={kp}, kd={kd}")
    
    iteration = 0
    while True:
        q_target += step_rad
        controller.controlMIT(motor, kp, kd, q_target, 0.0, tau_cmd)
        time.sleep(0.005)
        
        vel = motor.getVelocity()
        tau = motor.getTorque()
        pos = motor.getPosition()
        
        if iteration % 50 == 0:
            print(f"  [{iteration:4d}] pos={pos:.4f}, vel={vel:.4f}, tau={tau:.4f}")
        
        if np.abs(vel) < dq_th and np.abs(tau) > tau_th:
            delta_rad = pos - q_start
            delta_deg = np.rad2deg(delta_rad)
            print(f"[INFO] mechanical stop ({joint_id.name}): {delta_rad:.4f} rad / {delta_deg:.2f}°")
            return float(delta_rad)
        
        iteration += 1
        if iteration > 2000:  # Safety timeout
            print(f"[WARNING] Timeout on {joint_id.name}")
            return float(motor.getPosition() - q_start)

def interpolate(controller, motor_dict, joint_id: JointID, delta_rad,
                kp=52.0, kd=1.5, torque_assist=0.0, interp_time=2.0):
    """Linear interpolation to target position (matching original)."""
    idx = int(joint_id)
    motor_idx = idx + 1
    motor = motor_dict[motor_idx]
    
    q0 = motor.getPosition()
    q1 = q0 + float(delta_rad)
    n_steps = 500
    dt = interp_time / n_steps
    
    # Torque assist in direction of motion (from original)
    tau = np.copysign(1.0, q1 - q0) * abs(torque_assist)
    
    for i in range(n_steps + 1):
        alpha = i / n_steps
        q = q0 + (q1 - q0) * alpha
        controller.controlMIT(motor, kp, kd, q, 0.0, tau)
        time.sleep(dt)
    
    print(f"[INFO] interpolated {joint_id.name}: {np.rad2deg(delta_rad):.2f}°")


def calc_delta_to_zero_pos_joint(initial_rad: float,
                                 ideal_limit_rad: float,
                                 delta_to_stop_rad: float,
                                 joint_id: JointID) -> float:
    """Relative delta from 'hit position' to 'ideal limit', with per-joint sign (from original)."""
    q_hit = initial_rad + delta_to_stop_rad
    delta_to_ideal = ideal_limit_rad - q_hit
    return float(JOINT_SIGN.get(joint_id, 1.0) * delta_to_ideal)


def hold_all_joints(controller, motor_dict, positions):
    """Hold all joints at specified positions with per-joint gains (matching original)."""
    for idx in range(8):
        motor = motor_dict[idx + 1]
        kp = HOLD_KP[idx]
        kd = HOLD_KD[idx]
        q = positions[idx]
        controller.controlMIT(motor, kp, kd, q, 0.0, 0.0)

def main():
    parser = argparse.ArgumentParser(description='Zero-position calibration using DM_CAN')
    parser.add_argument('--canport', type=str, default='can0')
    parser.add_argument('--arm-side', type=str, default='right_arm', 
                        choices=['right_arm', 'left_arm'])
    args = parser.parse_args()
    
    print(f"Calibration for {args.arm_side} on {args.canport}")
    print("="*60)
    
    # Select motor dictionary based on CAN port
    motor_dict = Motor_CAN1 if args.canport == 'can1' else Motor_CAN0
    
    # Initialize
    controller = SocketCANMotorControl(can_interface=args.canport)
    
    # Add all motors
    for motor in motor_dict.values():
        controller.addMotor(motor)
    
    print(f"✓ Added {len(motor_dict)} motors")
    
    # Enable all motors
    print("\nEnabling motors...")
    for motor in motor_dict.values():
        controller.enable(motor)
        time.sleep(0.01)
    print("✓ Motors enabled")
    time.sleep(0.5)
    
    # Switch to MIT mode (with timeout handling)
    print("\nSwitching to MIT control mode...")
    for idx, motor in motor_dict.items():
        success = controller.switchControlMode(motor, Control_Type.MIT)
        if not success:
            print(f"  ⚠ Motor {idx} ({JointID(idx-1).name}): mode switch timeout (continuing anyway)")
        else:
            print(f"  ✓ Motor {idx} ({JointID(idx-1).name}): MIT mode set")
        time.sleep(0.05)
    print("✓ MIT mode commands sent")
    time.sleep(0.5)
    
    # Read initial positions
    print("\nReading initial positions...")
    initial_positions = []
    for joint_id in range(8):
        motor = motor_dict[joint_id + 1]
        pos = motor.getPosition()
        initial_positions.append(pos)
        print(f"  {JointID(joint_id).name}: {pos:.4f} rad")
    
    # Hold current pose with proper gains (matching original)
    print("\nHolding initial pose with per-joint gains...")
    for _ in range(20):  # Hold for 1 second
        hold_all_joints(controller, motor_dict, initial_positions)
        time.sleep(0.05)
    print("✓ Holding pose")
    
    try:
        print("\n" + "="*60)
        print("Starting calibration sequence...")
        print("="*60)
        print("\n⚠️  WARNING: Motors will move to mechanical stops!")
        print("This will take several minutes to complete.")
        print("Press Ctrl+C within 5 seconds to abort...")
        
        # Hold during countdown (matching original)
        for countdown in range(5, 0, -1):
            print(f"  {countdown}...")
            for _ in range(20):
                hold_all_joints(controller, motor_dict, initial_positions)
                time.sleep(0.05)
        
        if args.arm_side == 'right_arm':
            print("\n🔧 RIGHT ARM CALIBRATION SEQUENCE")
            
            # 1. Gripper
            print("\n1. Calibrating gripper...")
            d_grip = bump_to_limit(controller, motor_dict, JointID.GRIPPER, step_deg=0.2)
            time.sleep(0.5)
            
            # 2. J4 (move down)
            print("\n2. Calibrating J4 (elbow down)...")
            d_j4 = bump_to_limit(controller, motor_dict, JointID.J4, step_deg=-0.2)
            time.sleep(0.5)
            
            # 3. J2 adjust
            print("\n3. Adjusting J2...")
            interpolate(controller, motor_dict, JointID.J2, np.deg2rad(5), interp_time=0.4)
            time.sleep(0.5)
            
            # 4. J3
            print("\n4. Calibrating J3...")
            d_j3 = bump_to_limit(controller, motor_dict, JointID.J3)
            time.sleep(0.5)
            
            # Move J3 back (using mech_lim from original)
            interpolate(controller, motor_dict, JointID.J3, -mech_lim[JointID.J3][1], interp_time=1.0)
            time.sleep(0.5)
            
            # 5. J2 adjust back
            interpolate(controller, motor_dict, JointID.J2, -np.deg2rad(5), interp_time=0.4)
            time.sleep(0.5)
            
            # 6. J4 to 90°
            interpolate(controller, motor_dict, JointID.J4, np.pi/2.0)
            time.sleep(0.5)
            
            # 7. J5 (using mech_lim from original)
            print("\n5. Calibrating J5...")
            d_j5 = bump_to_limit(controller, motor_dict, JointID.J5)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J5, -mech_lim[JointID.J5][1])
            time.sleep(0.5)
            
            # 8. J6 (using mech_lim from original)
            print("\n6. Calibrating J6...")
            d_j6 = bump_to_limit(controller, motor_dict, JointID.J6)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J6, -mech_lim[JointID.J6][1])
            time.sleep(0.5)
            
            # 9. J7 (using mech_lim from original)
            print("\n7. Calibrating J7...")
            d_j7 = bump_to_limit(controller, motor_dict, JointID.J7)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J7, -mech_lim[JointID.J7][1])
            time.sleep(0.5)
            
            # 10. J2 second limit (kp=180, kd=2.0 from original)
            print("\n8. Calibrating J2 (second limit)...")
            d_j2 = bump_to_limit(controller, motor_dict, JointID.J2, step_deg=-0.2)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J2, np.deg2rad(10), interp_time=0.9, kp=180, kd=2.0)
            time.sleep(0.5)
            
            # 11. J1 (kp=180, kd=2.1 from original)
            print("\n9. Calibrating J1 (base)...")
            d_j1 = bump_to_limit(controller, motor_dict, JointID.J1, step_deg=-0.2, kp=180, kd=2.1)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J1, np.deg2rad(80))
            time.sleep(0.5)
            
            # Final positioning
            print("\n10. Final positioning...")
            interpolate(controller, motor_dict, JointID.J4, -np.pi/2.0)
            time.sleep(2.5)  # 2.5s from original
            
            # Ideal positions for zero calculation (from original)
            ideal = {
                JointID.J7: np.pi/2.0,
                JointID.J6: np.pi/4.0,
                JointID.J5: np.pi/2.0,
                JointID.J4: 0.0,
                JointID.J3: np.pi/2.0,
                JointID.J2: np.deg2rad(-10),
                JointID.J1: np.deg2rad(-80),
            }
            deltas = [d_j1, d_j2, d_j3, d_j4, d_j5, d_j6, d_j7, d_grip]
            
        else:  # left_arm
            print("\n🔧 LEFT ARM CALIBRATION SEQUENCE")
            
            # 1. Gripper
            print("\n1. Calibrating gripper...")
            d_grip = bump_to_limit(controller, motor_dict, JointID.GRIPPER, step_deg=0.2)
            time.sleep(0.5)
            
            # 2. J4 (move down)
            print("\n2. Calibrating J4 (elbow down)...")
            d_j4 = bump_to_limit(controller, motor_dict, JointID.J4, step_deg=-0.2)
            time.sleep(0.5)
            
            # 3. J2 adjust
            print("\n3. Adjusting J2...")
            interpolate(controller, motor_dict, JointID.J2, -np.deg2rad(5), interp_time=0.4)
            time.sleep(0.5)
            
            # 4. J3 (SAME direction as right arm - from original!)
            print("\n4. Calibrating J3...")
            d_j3 = bump_to_limit(controller, motor_dict, JointID.J3)  # Default +0.2
            time.sleep(0.5)
            
            # Move J3 back (using mech_lim from original)
            interpolate(controller, motor_dict, JointID.J3, -mech_lim[JointID.J3][1], interp_time=1.0)
            time.sleep(0.5)
            
            # 5. J2 adjust back
            interpolate(controller, motor_dict, JointID.J2, np.deg2rad(5), interp_time=0.4)
            time.sleep(0.5)
            
            # 6. J4 to 90°
            interpolate(controller, motor_dict, JointID.J4, np.pi/2.0)
            time.sleep(0.5)
            
            # 7. J5 (SAME direction as right arm - from original!)
            print("\n5. Calibrating J5...")
            d_j5 = bump_to_limit(controller, motor_dict, JointID.J5)  # Default +0.2
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J5, -mech_lim[JointID.J5][1])
            time.sleep(0.5)
            
            # 8. J6 (SAME direction as right arm - from original!)
            print("\n6. Calibrating J6...")
            d_j6 = bump_to_limit(controller, motor_dict, JointID.J6)  # Default +0.2
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J6, -mech_lim[JointID.J6][1])
            time.sleep(0.5)
            
            # 9. J7 (SAME direction as right arm - from original!)
            print("\n7. Calibrating J7...")
            d_j7 = bump_to_limit(controller, motor_dict, JointID.J7)  # Default +0.2
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J7, -mech_lim[JointID.J7][1])
            time.sleep(0.5)
            
            # 10. J2 second limit (opposite direction from right - from original)
            print("\n8. Calibrating J2 (second limit)...")
            d_j2 = bump_to_limit(controller, motor_dict, JointID.J2)  # Default +0.2 (opposite of right)
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J2, -np.deg2rad(10), interp_time=0.9, kp=180, kd=2.0)
            time.sleep(0.5)
            
            # 11. J1 (opposite direction from right - from original)
            print("\n9. Calibrating J1 (base)...")
            d_j1 = bump_to_limit(controller, motor_dict, JointID.J1, kp=180, kd=2.1)  # Default +0.2
            time.sleep(0.5)
            interpolate(controller, motor_dict, JointID.J1, -np.deg2rad(80))
            time.sleep(0.5)
            
            # Final positioning
            print("\n10. Final positioning...")
            interpolate(controller, motor_dict, JointID.J4, -np.pi/2.0)
            time.sleep(2.5)  # 2.5s from original
            
            # Ideal positions for zero calculation (from original)
            ideal = {
                JointID.J7: np.pi/2.0,
                JointID.J6: np.pi/4.0,
                JointID.J5: np.pi/2.0,
                JointID.J4: 0.0,
                JointID.J3: np.pi/2.0,
                JointID.J2: np.deg2rad(10),   # Note: opposite sign from right arm
                JointID.J1: np.deg2rad(80),   # Note: opposite sign from right arm
            }
            deltas = [d_j1, d_j2, d_j3, d_j4, d_j5, d_j6, d_j7, d_grip]
        
        print("\n" + "="*60)
        print("CALIBRATION RESULTS")
        print("="*60)
        
        # Raw deltas
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "GRIPPER"]
        print("\nRaw deltas (traveled to mechanical stop):")
        for name, delta in zip(joint_names, deltas):
            print(f"  {name}: {delta:8.4f} rad ({np.rad2deg(delta):7.2f}°)")
        
        # Compute ideal deltas per joint (from original)
        joint_order = [JointID.J1, JointID.J2, JointID.J3, JointID.J4,
                       JointID.J5, JointID.J6, JointID.J7, JointID.GRIPPER]
        
        print("\nIdeal zero position offsets:")
        ideal_delta_map = {}
        for j, d in zip(joint_order, deltas):
            if j == JointID.GRIPPER:
                print(f"  {j.name}: (skipped)")
                continue
            val = calc_delta_to_zero_pos_joint(
                initial_rad=initial_positions[int(j)],
                ideal_limit_rad=ideal[j],
                delta_to_stop_rad=d,
                joint_id=j
            )
            ideal_delta_map[j] = val
            print(f"  {j.name}: {val:8.6f} rad")
        
        # Compute absolute goal positions
        arm_goal_abs = [initial_positions[i] + ideal_delta_map.get(JointID(i), 0.0) 
                        for i in range(7)]
        print("\nAbsolute goal positions (initial + ideal_delta):")
        for i, goal in enumerate(arm_goal_abs):
            print(f"  {JointID(i).name}: {goal:8.4f} rad ({np.rad2deg(goal):7.2f}°)")
        
        print("\n✓ Calibration complete!")
        
        # Ask user if they want to write zero positions
        print("\n" + "="*60)
        print("WRITE ZERO POSITIONS TO FLASH MEMORY?")
        print("="*60)
        print("This will set the current positions as the zero reference.")
        print("This operation is PERMANENT until you calibrate again.")
        response = input("\nType 'YES' to write zeros, anything else to skip: ")
        
        if response.strip().upper() == 'YES':
            # Disable before writing zeros (matching original)
            print("\nDisabling motors before writing zeros...")
            for motor in motor_dict.values():
                controller.disable(motor)
            time.sleep(0.1)
            
            print("Writing zero positions to all motors...")
            for idx, motor in motor_dict.items():
                print(f"  Setting zero for motor {idx}...")
                controller.set_zero_position(motor)
                time.sleep(0.1)
            print("✓ Zero positions written to flash memory")
            print("\n⚠️  Power cycle the motors to apply the new zero positions!")
        else:
            print("\n⚠️  Zero positions NOT written. Run calibration again when ready.")
        
    except KeyboardInterrupt:
        print("\n\n[INFO] Calibration aborted by user")
    finally:
        print("\nDisabling motors...")
        for motor in motor_dict.values():
            controller.disable(motor)
        print("✓ Motors disabled safely")

if __name__ == "__main__":
    main()
