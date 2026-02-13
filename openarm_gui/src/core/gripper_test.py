import os
import sys
import time
import numpy as np

# 프로젝트 루트를 PYTHONPATH에 추가하여 core 모듈을 임포트할 수 있게 함
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "../../"))
if project_root not in sys.path:
    sys.path.append(project_root)

from src.core.can_controller import OpenArmCANController

def main():
    print("="*50)
    print("   OpenArm Gripper MIT Control Test Tool   ")
    print("="*50)
    
    # 1. 컨트롤러 초기화
    controller = OpenArmCANController()
    
    # 2. CAN 연결
    print("\n[1/3] CAN 인터페이스 연결 중...")
    if not controller.connect():
        print("❌ CAN 연결 실패. 프로그램을 종료합니다.")
        return
    
    # 3. 제어 루프 시작
    print("[2/3] 제어 루프 시작 중...")
    controller.start()
    
    # 초기 설정 및 안전값
    # 사용자의 요청에 따라 작은 리스크 값으로 시작 (KP=1.0, KD=0.05)
    target_arm = 'left' # 기본 테스트 대상
    current_kp = 16.0
    current_kd = 0.2
    target_pos = 0.0 # 0.0 = Close
    
    controller.control_gains['gripper']['kp'] = current_kp
    controller.control_gains['gripper']['kd'] = current_kd
    
    print(f"\n[3/3] 초기화 완료. 현재 대상: {target_arm} 암 그리퍼 (ID 8)")
    print(f"기본값: KP={current_kp}, KD={current_kd}, Target={target_pos}")
    print("\n명령어 안내:")
    print("  p [val]  : 목표 위치 설정 (meters, 0.0=Close, 0.044=Open)")
    print("  open     : 그리퍼 열기 (p 0.044)")
    print("  close    : 그리퍼 닫기 (p 0.0)")
    print("  kp [val] : KP 값 변경")
    print("  kd [val] : KD 값 변경")
    print("  arm [l/r]: 테스트 대상 암 변경 (left/right)")
    print("  zero     : 현재 위치를 0(Close)으로 설정")
    print("  s        : 현재 상태 출력")
    print("  q        : 종료 (모터 비활성화)")
    print("="*50)

    try:
        while True:
            # 현재 상태 읽기
            state = controller.get_robot_state()
            arm_state = state.get(target_arm, {})
            current_q = arm_state.get('joints', [0]*8)[7]
            current_tau = arm_state.get('effort', [0]*8)[7]

            cmd_input = input(f"[{target_arm}] P:{target_pos:.3f}, KP:{current_kp:.1f}, KD:{current_kd:.2f} | Current Q:{current_q:.3f} > ").strip().split()
            
            if not cmd_input:
                continue
            
            action = cmd_input[0].lower()
            
            if action == 'q':
                print("종료 중...")
                break
            
            elif action == 's':
                print(f"\n--- {target_arm} Gripper Status ---")
                print(f"Target Position: {target_pos:.4f} rad")
                print(f"Current Position: {current_q:.4f} rad")
                print(f"Current Torque: {current_tau:.4f} Nm")
                print(f"Gains: KP={current_kp}, KD={current_kd}")
                print("-" * 30)

            elif action == 'arm':
                if len(cmd_input) > 1:
                    side = cmd_input[1].lower()
                    if side in ['l', 'left']:
                        target_arm = 'left'
                    elif side in ['r', 'right']:
                        target_arm = 'right'
                    print(f"대상을 {target_arm}으로 변경했습니다.")
                else:
                    print("사용법: arm [l/r]")

            elif action == 'p':
                if len(cmd_input) > 1:
                    try:
                        target_pos = float(cmd_input[1])
                        # 그리퍼 타겟 설정 (ID 8)
                        controller.set_gripper_position(target_arm, target_pos)
                        print(f"목표 위치를 {target_pos}로 설정했습니다.")
                    except ValueError:
                        print("올바른 숫자를 입력하세요.")
                else:
                    print("사용법: p [위치]")

            elif action == 'open':
                target_pos = 0.044
                controller.set_gripper_position(target_arm, target_pos)
                print(f"그리퍼를 엽니다. (목표: {target_pos}m)")

            elif action == 'close':
                target_pos = 0.0
                controller.set_gripper_position(target_arm, target_pos)
                print(f"그리퍼를 닫습니다. (목표: {target_pos})")

            elif action == 'kp':
                if len(cmd_input) > 1:
                    try:
                        current_kp = float(cmd_input[1])
                        controller.control_gains['gripper']['kp'] = current_kp
                        print(f"KP를 {current_kp}로 설정했습니다.")
                    except ValueError:
                        print("올바른 숫자를 입력하세요.")

            elif action == 'kd':
                if len(cmd_input) > 1:
                    try:
                        current_kd = float(cmd_input[1])
                        controller.control_gains['gripper']['kd'] = current_kd
                        print(f"KD를 {current_kd}로 설정했습니다.")
                    except ValueError:
                        print("올바른 숫자를 입력하세요.")
            
            elif action == 'zero':
                confirm = input(f"⚠️ 정말로 {target_arm} 그리퍼의 현재 위치를 0으로 설정하시겠습니까? (y/n): ").lower()
                if confirm == 'y':
                    controller.zero_gripper(target_arm)
                    # Zero 설정 후 타겟 위치도 0으로 초기화하여 급격한 움직임 방지
                    target_pos = 0.0
                    controller.set_gripper_position(target_arm, target_pos)
                    print(f"현재 위치가 0(Close)으로 설정되었습니다. 목표 위치도 0.0으로 초기화되었습니다.")
            
            else:
                print("알 수 없는 명령어입니다.")

    except KeyboardInterrupt:
        print("\n중단됨.")
    finally:
        print("모터를 정지하고 연결을 해제합니다...")
        controller.stop()
        print("완료.")

if __name__ == "__main__":
    main()
