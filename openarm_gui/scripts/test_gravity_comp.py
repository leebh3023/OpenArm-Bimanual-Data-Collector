#!/usr/bin/env python3
"""
test_gravity_comp.py: Pinocchio 중력보상 인터랙티브 테스트 스크립트.

CAN Controller를 통해 로봇 상태를 읽고, Pinocchio로 계산된 중력 토크를
MIT 모드의 Feed-forward Torque (tau_ff)로 전달하여 중력 보상을 테스트합니다.

사용법:
    .venv/bin/python scripts/test_gravity_comp.py
"""

import os
import sys
import time
import threading
import numpy as np

# 프로젝트 루트를 PYTHONPATH에 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

from src.core.can_controller import OpenArmCANController
from src.core.gravity_compensator import GravityCompensator


class GravityCompensationTest:
    """중력 보상 테스트 매니저"""

    def __init__(self):
        self.controller = OpenArmCANController()
        self.compensator = None  # Pinocchio 모델 로드 후 초기화

        self.target_arm = 'left'
        self.enabled = False       # 보상 활성화 여부
        self.comp_kd = 0.5         # 보상 모드에서 사용할 KD (댐핑)
        self._running = False
        self._comp_thread = None

        # 실시간 모니터링 데이터
        self._last_tau_gravity = np.zeros(8)
        self._last_tau_actual = np.zeros(8)

    def start(self):
        """시스템 초기화 및 시작"""
        print("=" * 55)
        print("   OpenArm Gravity Compensation Test (Pinocchio)")
        print("=" * 55)

        # 1. Pinocchio 모델 로드
        print("\n[1/3] Pinocchio 동역학 모델 로드 중...")
        try:
            self.compensator = GravityCompensator(scale=0.0)
            self.compensator.print_info()
        except Exception as e:
            print(f"❌ Pinocchio 모델 로드 실패: {e}")
            return False

        # 2. CAN 연결
        print("\n[2/3] CAN 인터페이스 연결 중...")
        if not self.controller.connect():
            print("❌ CAN 연결 실패.")
            return False

        # 3. 제어 루프 시작
        print("[3/3] 제어 루프 시작...")
        self.controller.start()

        # FreeDrive 모드로 시작 (양쪽 팔 모두)
        self.controller.enable_freedrive('left')
        self.controller.enable_freedrive('right')

        # 중력 보상 스레드 시작
        self._running = True
        self._comp_thread = threading.Thread(target=self._compensation_loop, daemon=True)
        self._comp_thread.start()

        print("\n✅ 초기화 완료. 중력 보상은 OFF 상태입니다.")
        print("   'on' 명령으로 활성화하세요. (scale을 먼저 낮게 설정 권장)")
        return True

    def stop(self):
        """안전 종료"""
        print("\n정리 중...")
        self._running = False
        self.enabled = False

        if self._comp_thread and self._comp_thread.is_alive():
            self._comp_thread.join(timeout=2.0)

        self.controller.stop()
        print("완료.")

    def _compensation_loop(self):
        """
        백그라운드 중력 보상 루프 (100Hz).
        
        활성화 시: KP=0, KD=comp_kd, tau_ff=중력토크 로 MIT 명령 전송.
        비활성화 시: 기본 passive 모드 (FreeDrive).
        """
        rate = 0.01  # 100Hz

        while self._running:
            start_time = time.time()

            if self.enabled and self.compensator is not None:
                try:
                    self._send_gravity_compensation()
                except Exception as e:
                    print(f"\n[ERROR] 보상 루프 오류: {e}")
                    self.enabled = False
                    print("[SAFETY] 보상이 자동으로 비활성화되었습니다.")

            elapsed = time.time() - start_time
            sleep_time = max(0, rate - elapsed)
            time.sleep(sleep_time)

    def _send_gravity_compensation(self):
        """
        현재 관절 상태를 읽고, 중력 보상 토크를 계산하여 MIT 명령으로 전송.
        """
        state = self.controller.get_robot_state()

        # 양쪽 팔의 현재 상태 가져오기
        left_joints = state['left']['joints']    # 8-element
        right_joints = state['right']['joints']  # 8-element
        
        # 대상 팔 선택
        if self.target_arm == 'left':
            arm_joints = left_joints
            other_joints = right_joints
        else:
            arm_joints = right_joints
            other_joints = left_joints

        # Pinocchio 중력 토크 계산
        tau_ff_8 = self.compensator.get_feedforward_torques(
            self.target_arm, arm_joints, other_joints
        )

        # 모니터링용 저장
        self._last_tau_gravity = tau_ff_8.copy()
        self._last_tau_actual = state[self.target_arm]['effort'].copy()

        # MIT 명령 전송: KP=0, KD=comp_kd, q_des=현재위치, tau_ff=중력토크
        bus = self.controller.buses.get(self.target_arm)
        if bus is None:
            return

        arm_configs = self.controller.motor_configs.get(self.target_arm, {})
        for i in range(7):  # J1~J7만 (그리퍼 제외)
            motor_id = i + 1
            if motor_id not in arm_configs:
                continue

            self.controller.send_mit_command(
                arm=self.target_arm,
                motor_id=motor_id,
                kp=0.0,                    # 위치 추종 없음
                kd=self.comp_kd,           # 적절한 댐핑
                q_des=arm_joints[i],        # 현재 위치 (drift 방지)
                dq_des=0.0,
                tau_ff=tau_ff_8[i]          # 중력 보상 토크
            )

    def run_interactive(self):
        """인터랙티브 CLI 루프"""
        self._print_help()

        try:
            while True:
                state = self.controller.get_robot_state()
                arm_q = state[self.target_arm]['joints']

                status = "🟢 ON" if self.enabled else "🔴 OFF"
                scale_str = f"{self.compensator.scale:.2f}" if self.compensator else "N/A"

                prompt = (f"[{self.target_arm}] {status} "
                         f"Scale:{scale_str} "
                         f"KD:{self.comp_kd:.2f} > ")
                cmd_input = input(prompt).strip().split()

                if not cmd_input:
                    continue

                action = cmd_input[0].lower()

                if action == 'q':
                    print("종료 중...")
                    break

                elif action == 'on':
                    if self.compensator.scale == 0.0:
                        print("⚠️  Scale이 0.0입니다. 먼저 'scale 0.3' 등으로 설정하세요.")
                    else:
                        self.enabled = True
                        # FreeDrive 모드 해제 (MIT 명령 직접 전송하므로)
                        # commands를 None으로 유지하여 _send_mit_commands가 passive로 보내는 것을 방지
                        print(f"✅ 중력 보상 활성화 (Scale: {self.compensator.scale:.2f})")

                elif action == 'off':
                    self.enabled = False
                    # FreeDrive 복귀
                    self.controller.enable_freedrive(self.target_arm)
                    print("🔴 중력 보상 비활성화 (FreeDrive 복귀)")

                elif action == 'scale':
                    if len(cmd_input) > 1:
                        try:
                            val = float(cmd_input[1])
                            self.compensator.set_scale(val)
                        except ValueError:
                            print("올바른 숫자를 입력하세요. 예: scale 0.5")
                    else:
                        print(f"현재 Scale: {self.compensator.scale:.2f}")
                        print("사용법: scale [0.0 ~ 1.0]")

                elif action == 'kd':
                    if len(cmd_input) > 1:
                        try:
                            self.comp_kd = float(cmd_input[1])
                            print(f"KD를 {self.comp_kd:.2f}로 설정했습니다.")
                        except ValueError:
                            print("올바른 숫자를 입력하세요.")
                    else:
                        print(f"현재 KD: {self.comp_kd:.2f}")

                elif action == 'arm':
                    if len(cmd_input) > 1:
                        side = cmd_input[1].lower()
                        if side in ['l', 'left']:
                            self.target_arm = 'left'
                        elif side in ['r', 'right']:
                            self.target_arm = 'right'
                        print(f"대상 팔: {self.target_arm}")
                    else:
                        print("사용법: arm [l/r]")

                elif action == 's':
                    self._print_status(state)

                elif action == 'help':
                    self._print_help()

                else:
                    print("알 수 없는 명령어입니다. 'help'를 입력하세요.")

        except KeyboardInterrupt:
            print("\n중단됨.")
        finally:
            self.stop()

    def _print_status(self, state):
        """현재 상태 및 토크 비교 출력"""
        arm_q = state[self.target_arm]['joints']
        arm_v = state[self.target_arm]['velocity']
        arm_tau = state[self.target_arm]['effort']

        print(f"\n{'='*60}")
        print(f"  [{self.target_arm.upper()}] 상태 및 중력 보상 정보")
        print(f"{'='*60}")
        print(f"  Scale: {self.compensator.scale:.2f} | "
              f"KD: {self.comp_kd:.2f} | "
              f"Status: {'ON' if self.enabled else 'OFF'}")
        print(f"{'─'*60}")
        print(f"  {'Joint':>7} {'Position':>10} {'Velocity':>10} "
              f"{'Actual τ':>10} {'Gravity τ':>10} {'Diff':>10}")
        print(f"{'─'*60}")

        for i in range(7):
            diff = arm_tau[i] - self._last_tau_gravity[i]
            print(f"  J{i+1:>5} {arm_q[i]:>10.4f} {arm_v[i]:>10.4f} "
                  f"{arm_tau[i]:>10.4f} {self._last_tau_gravity[i]:>10.4f} "
                  f"{diff:>10.4f}")

        # 그리퍼 상태도 표시
        print(f"  {'Grip':>7} {arm_q[7]:>10.4f} {arm_v[7]:>10.4f} "
              f"{arm_tau[7]:>10.4f} {'N/A':>10} {'N/A':>10}")
        print(f"{'='*60}")

        # Zero 자세에서의 이론적 중력 토크도 참고용으로 표시
        if not self.enabled:
            tau_at_current = self.compensator.get_feedforward_torques(
                self.target_arm, arm_q,
                state['right' if self.target_arm == 'left' else 'left']['joints']
            )
            print(f"\n  [참고] 현재 자세에서의 계산된 중력 토크:")
            for i in range(7):
                print(f"    J{i+1}: {tau_at_current[i]:>10.4f} Nm")
        print()

    def _print_help(self):
        """도움말 출력"""
        print(f"\n{'─'*55}")
        print("  명령어:")
        print("    on          중력 보상 활성화")
        print("    off         중력 보상 비활성화 (FreeDrive 복귀)")
        print("    scale [val] 보상 스케일 설정 (0.0~1.0, 기본 0.0)")
        print("    kd [val]    댐핑(KD) 설정 (기본 0.5)")
        print("    arm [l/r]   대상 팔 변경 (left/right)")
        print("    s           현재 상태 + 토크 비교 출력")
        print("    help        이 도움말 출력")
        print("    q           안전 종료")
        print(f"{'─'*55}")
        print("  ⚠️  주의: scale을 낮은 값(0.3)부터 시작하세요!")
        print()


def main():
    test = GravityCompensationTest()

    if not test.start():
        print("초기화 실패. 프로그램을 종료합니다.")
        return

    test.run_interactive()


if __name__ == "__main__":
    main()
