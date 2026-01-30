import time
import threading
from typing import Dict, Any, Optional
import numpy as np

try:
    import can
except ImportError:
    can = None

class OpenArmCANController(threading.Thread):
    """
    CAN 통신을 통해 OpenArm Bimanual 로봇을 제어하고 상태를 수신하는 클래스.
    별도의 스레드에서 실시간 제어 루프를 구동합니다.
    """
    
    def __init__(self, channel: str = 'can0', bustype: str = 'socketcan', bitrate: int = 1000000):
        super().__init__(daemon=True)
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        
        self.bus: Optional[can.BusABC] = None
        self._running = False
        self._lock = threading.Lock()
        
        # 로봇 상태 데이터 (Thread-safe)
        self.robot_state = {
            'left': {'joints': np.zeros(7), 'velocity': np.zeros(7), 'effort': np.zeros(7)},
            'right': {'joints': np.zeros(7), 'velocity': np.zeros(7), 'effort': np.zeros(7)},
            'timestamp': 0.0
        }
        
        # 제어 명령 데이터
        self.commands = {
            'left': np.zeros(7),
            'right': np.zeros(7)
        }
        
        self.logger = None # 메인 앱에서 로거 주입 가능

    def connect(self) -> bool:
        """CAN 버스에 접속합니다."""
        if can is None:
            print("Warning: 'python-can' module not found. Running in simulation mode.")
            self._running = True
            return True
            
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype=self.bustype, bitrate=self.bitrate)
            self._running = True
            return True
        except Exception as e:
            print(f"Failed to connect to CAN: {e}")
            return False

    def stop(self):
        """제어 루프를 중지하고 연결을 해제합니다."""
        self._running = False
        if self.bus:
            self.bus.shutdown()

    def run(self):
        """실시간 제어 루프 (예: 100Hz)"""
        rate = 0.01 # 100Hz
        while self._running:
            start_time = time.time()
            
            # 1. CAN 메시지 수신 및 상태 업데이트
            self._receive_states()
            
            # 2. 제어 명령 송신
            self._send_commands()
            
            # 3. 루프 타이밍 조절
            elapsed = time.time() - start_time
            sleep_time = max(0, rate - elapsed)
            time.sleep(sleep_time)

    def _receive_states(self):
        """로봇 팔로부터 CAN 데이터를 수신하여 내부 상태 업데이트"""
        if self.bus:
            # 실물 CAN 수신 로직 (예시)
            # msg = self.bus.recv(timeout=0.001)
            # if msg:
            #     self._parse_can_msg(msg)
            pass
        else:
            # 시뮬레이션 데이터 생성
            with self._lock:
                self.robot_state['timestamp'] = time.time()
                # 약간의 노이즈 추가
                self.robot_state['left']['joints'] += np.random.normal(0, 0.001, 7)
                self.robot_state['right']['joints'] += np.random.normal(0, 0.001, 7)

    def _send_commands(self):
        """현재 저장된 제어 명령을 CAN 메시지로 송신"""
        if self.bus:
            # 실물 CAN 송신 로직
            pass

    def get_robot_state(self) -> Dict[str, Any]:
        """UI에서 로봇 상태를 안전하게 읽어갈 수 있도록 반환"""
        with self._lock:
            return self.robot_state.copy()

    def set_target_joints(self, arm: str, joints: np.ndarray):
        """특정 팔의 목표 관절 각도를 설정"""
        if arm in self.commands:
            with self._lock:
                self.commands[arm] = joints.copy()
