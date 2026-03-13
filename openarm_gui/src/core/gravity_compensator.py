"""
GravityCompensator: Pinocchio 기반 중력 보상 토크 계산 모듈.

Bimanual URDF 모델을 로드하여 현재 관절 각도로부터 중력 보상에 필요한
Feed-forward 토크를 실시간으로 계산합니다.
"""

import os
import numpy as np

try:
    import pinocchio as pin
except ImportError:
    pin = None
    print("Warning: 'pinocchio' (pin) 모듈을 찾을 수 없습니다. "
          ".venv/bin/pip install pin 으로 설치해주세요.")


class GravityCompensator:
    """
    Pinocchio RNEA 알고리즘 기반 중력 보상 토크 계산기.

    Bimanual URDF(18 DOF: 7 arm + 2 finger per side)를 로드하고,
    현재 관절 각도에 대한 중력 토크를 계산합니다.

    Attributes:
        model: Pinocchio 모델 객체
        data: Pinocchio 데이터 객체
        scale: 보상 스케일 팩터 (0.0 ~ 1.0, 안전 조절용)
    """

    # 기본 URDF 경로
    DEFAULT_URDF_PATH = "/home/rvlab/openarm_ws/openarm_bimanual_control.urdf"

    # Pinocchio 모델 내 Joint 이름 → 인덱스 매핑
    # buildModelFromUrdf 결과:
    #   0: universe (fixed)
    #   1-7:  openarm_left_joint1~7
    #   8-9:  openarm_left_finger_joint1~2
    #   10-16: openarm_right_joint1~7
    #   17-18: openarm_right_finger_joint1~2
    # Generalized coordinate q 인덱스 (0-indexed):
    LEFT_ARM_Q_INDICES = list(range(0, 7))      # q[0:7]
    LEFT_FINGER_Q_INDICES = list(range(7, 9))    # q[7:9]
    RIGHT_ARM_Q_INDICES = list(range(9, 16))     # q[9:16]
    RIGHT_FINGER_Q_INDICES = list(range(16, 18)) # q[16:18]

    def __init__(self, urdf_path: str = None, scale: float = 1.0):
        """
        Args:
            urdf_path: URDF 파일 경로. None이면 기본 경로 사용.
            scale: 중력 보상 스케일 (0.0=보상 비활성화, 1.0=100% 보상)
        """
        if pin is None:
            raise RuntimeError(
                "Pinocchio가 설치되어 있지 않습니다. "
                ".venv/bin/pip install pin 으로 설치해주세요."
            )

        self.urdf_path = urdf_path or self.DEFAULT_URDF_PATH
        self.scale = np.clip(scale, 0.0, 1.0)

        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {self.urdf_path}")

        # Pinocchio 모델 생성
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        print(f"[GravityCompensator] URDF 로드 완료: {self.urdf_path}")
        print(f"  - DOF: {self.model.nq} (nq), {self.model.nv} (nv)")
        print(f"  - Gravity: {self.model.gravity.linear}")
        print(f"  - Scale: {self.scale}")

    def compute_full_gravity_torque(self, q_full: np.ndarray) -> np.ndarray:
        """
        전체 18-DOF 관절에 대한 중력 토크를 계산합니다.

        Args:
            q_full: 18-element 전체 관절 각도 배열

        Returns:
            18-element 중력 보상 토크 (스케일 적용 후)
        """
        assert len(q_full) == self.model.nq, \
            f"입력 크기 불일치: {len(q_full)} != {self.model.nq}"

        # computeGeneralizedGravity: G(q) = RNEA(q, 0, 0)
        tau_gravity = pin.computeGeneralizedGravity(self.model, self.data, q_full)
        return np.array(tau_gravity) * self.scale

    def compute_arm_gravity_torque(self, arm: str,
                                    arm_joints_7dof: np.ndarray,
                                    other_arm_joints_7dof: np.ndarray = None
                                    ) -> np.ndarray:
        """
        지정된 팔(left/right)의 7-DOF 관절에 대한 중력 토크를 계산합니다.

        두 팔을 포함한 전체 18-DOF 모델에서 중력 토크를 계산하고,
        해당 팔의 7개 관절 토크만 반환합니다.

        Args:
            arm: 'left' 또는 'right'
            arm_joints_7dof: 해당 팔의 7개 관절 각도 (J1~J7)
            other_arm_joints_7dof: 반대편 팔의 7개 관절 각도 (None이면 0으로 가정)

        Returns:
            7-element 중력 보상 토크 배열
        """
        # 전체 q 벡터 구성 (18 DOF)
        q_full = np.zeros(self.model.nq)

        if arm == 'left':
            q_full[0:7] = arm_joints_7dof[:7]
            if other_arm_joints_7dof is not None:
                q_full[9:16] = other_arm_joints_7dof[:7]
        elif arm == 'right':
            q_full[9:16] = arm_joints_7dof[:7]
            if other_arm_joints_7dof is not None:
                q_full[0:7] = other_arm_joints_7dof[:7]
        else:
            raise ValueError(f"Unknown arm: {arm}. Use 'left' or 'right'.")

        # 전체 모델에 대한 중력 토크 계산
        tau_full = self.compute_full_gravity_torque(q_full)

        # 해당 팔의 7 관절 토크만 추출
        if arm == 'left':
            return tau_full[self.LEFT_ARM_Q_INDICES]
        else:
            return tau_full[self.RIGHT_ARM_Q_INDICES]

    def get_feedforward_torques(self, arm: str,
                                 robot_state_8dof: np.ndarray,
                                 other_arm_state_8dof: np.ndarray = None
                                 ) -> np.ndarray:
        """
        CAN Controller 형태의 8-element 배열(J1~J7 + Gripper)을 받아,
        중력 보상 토크를 8-element 배열로 반환합니다 (그리퍼 토크는 0).

        Args:
            arm: 'left' 또는 'right'
            robot_state_8dof: 해당 팔의 8개 관절 각도 (CAN 컨트롤러 형식)
            other_arm_state_8dof: 반대편 팔의 8개 관절 각도 (None이면 0으로 가정)

        Returns:
            8-element 토크 배열 [J1~J7 중력토크, 0.0(gripper)]
        """
        arm_joints_7 = robot_state_8dof[:7]
        other_joints_7 = other_arm_state_8dof[:7] if other_arm_state_8dof is not None else None

        tau_7 = self.compute_arm_gravity_torque(arm, arm_joints_7, other_joints_7)

        # 8-element로 확장 (그리퍼 토크 = 0)
        tau_8 = np.zeros(8)
        tau_8[:7] = tau_7
        return tau_8

    def set_scale(self, scale: float):
        """보상 스케일 설정 (0.0 ~ 1.0)"""
        self.scale = np.clip(scale, 0.0, 1.0)
        print(f"[GravityCompensator] Scale 변경: {self.scale:.2f}")

    def print_info(self):
        """모델 정보 출력"""
        print(f"\n=== GravityCompensator 정보 ===")
        print(f"URDF: {self.urdf_path}")
        print(f"DOF: {self.model.nq}")
        print(f"Scale: {self.scale:.2f}")
        print(f"Gravity: {self.model.gravity.linear}")
        print(f"\nJoint 이름:")
        for i, name in enumerate(self.model.names):
            if i == 0:
                continue  # universe 건너뛰기
            print(f"  [{i}] {name}")
        print(f"{'='*30}")
