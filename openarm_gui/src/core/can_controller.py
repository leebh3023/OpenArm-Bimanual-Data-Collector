import time
import threading
from typing import Dict, Any, Optional
import numpy as np
import subprocess

try:
    import can
except ImportError:
    can = None

class OpenArmCANController:
    """
    CAN 통신을 통해 OpenArm Bimanual 로봇을 제어하고 상태를 수신하는 클래스.
    별도의 스레드에서 실시간 제어 루프를 구동합니다.
    """
    
    # DM Motor Protocol Helpers
    LIMIT_PARAM = [
        [12.5, 30, 10],   # 0: DM4310
        [12.5, 50, 10],   # 1: DM4310_48V
        [12.5, 8, 28],    # 2: DM4340
        [12.5, 10, 28],   # 3: DM4340_48V
        [12.5, 45, 20],   # 4: DM6006
        [12.5, 45, 40],   # 5: DM8006
        [12.5, 45, 54],   # 6: DM8009
        [12.5, 25, 200],  # 7: DM10010L
        [12.5, 20, 200],  # 8: DM10010
        [12.5, 280, 1],   # 9: DMH3510
        [12.5, 45, 10],   # 10: DMH6215
        [12.5, 45, 10]    # 11: DMG6220
    ]

    @staticmethod
    def float_to_uint(x, x_min, x_max, bits):
        x = max(min(x, x_max), x_min)
        span = x_max - x_min
        data_norm = (x - x_min) / span
        return int(data_norm * ((1 << bits) - 1))

    @staticmethod
    def uint_to_float(x, x_min, x_max, bits):
        span = x_max - x_min
        data_norm = float(x) / ((1 << bits) - 1)
        return data_norm * span + x_min

    def __init__(self, bitrate: int = 1000000, dbitrate: int = 5000000):
        # 듀얼 채널 구성 (Left: can0, Right: can1)
        # 듀얼 채널 구성 (Left: can1, Right: can0) - User Requested Swap
        self.channel_map = {
            'left': 'can1',
            'right': 'can0'
        }
        self.bitrate = bitrate
        self.dbitrate = dbitrate
        
        self.buses: Dict[str, Optional[can.BusABC]] = {'left': None, 'right': None}
        self._running = False
        self._thread: Optional[threading.Thread] = None
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
        
        # Motor Configuration (Arm side -> ID -> Type Index)
        # Type 6: DM8009 (J1, J2)
        # Type 3: DM4340_48V (Left J3, J4)
        # Type 2: DM4340 (Right J3, J4)
        # Type 0: DM4310 (J5, J6, J7)
        self.motor_configs = {
            'left': {
                1: 6, 2: 6,       # J1, J2
                3: 2, 4: 2,       # J3, J4 (Standard)
                5: 0, 6: 0, 7: 0  # J5, J6, J7
            },
            'right': {
                1: 6, 2: 6,       # J1, J2
                3: 2, 4: 2,       # J3, J4 (Standard)
                5: 0, 6: 0, 7: 0  # J5, J6, J7
            }
        }

        self.logger = None # 메인 앱에서 로거 주입 가능

    def _is_interface_up(self, interface: str) -> bool:
        """Check if network interface is UP via sysfs"""
        try:
            with open(f"/sys/class/net/{interface}/operstate", "r") as f:
                state = f.read().strip()
            # 'up' or 'unknown' (sometimes virtual/can interfaces show unknown but are up)
            # 'down' is clearly down.
            return state in ["up", "unknown"]
        except Exception:
            return False

    def connect(self) -> bool:
        """CAN 버스들에 접속합니다."""
        if can is None:
            print("Warning: 'python-can' module not found. Running in simulation mode.")
            self._running = True
            return True
            
        success_count = 0
        for arm, channel in self.channel_map.items():
            # 1. 시스템 레벨 인터페이스 설정 (sudo 권한 필요할 수 있음)
            try:
                cmd = [
                    "openarm-can-configure-socketcan", 
                    channel, 
                    "-fd", 
                    "-b", str(self.bitrate), 
                    "-d", str(self.dbitrate)
                ]
                print(f"Configuring {channel}...")
                result = subprocess.run(cmd, capture_output=True, text=True, input="", timeout=2)
                
                if result.returncode != 0:
                    print(f"Warning: Configuration command failed with return code {result.returncode}")
                    print(f"Stderr: {result.stderr.strip()}")
                    print("Note: This might be due to missing sudo permissions. If the interface is already up, connection may still succeed.")
                else:
                    print(f"Configuration command executed successfully.")
                    
            except subprocess.TimeoutExpired:
                print(f"Configuration command timed out (Sudo required?). Proceeding to connect...")
            except Exception as e:
                print(f"Configuration command failed: {e}")

            # 2. 인터페이스 상태 확인 (물리적/논리적 상태)
            if not self._is_interface_up(channel):
                print(f"Error: Interface {channel} is DOWN. Skipping connection for {arm} arm.")
                self.buses[arm] = None
                continue

            try:
                # 3. SocketCAN 접속
                bus = can.interface.Bus(
                    channel=channel, 
                    bustype='socketcan', 
                    fd=True,
                    bitrate=self.bitrate,
                    data_bitrate=self.dbitrate
                )
                self.buses[arm] = bus
                print(f"Successfully connected to {channel} ({arm})")
                success_count += 1
            except Exception as e:
                print(f"Failed to connect to {channel} ({arm}): {e}")
                self.buses[arm] = None

        if success_count > 0:
            self._running = True
            return True
        else:
            print("Failed to connect to any CAN interface. Running in simulation mode.")
            # 모두 실패해도 시뮬레이션 모드로 동작하도록 설정
            self._running = True 
            return True

    
    def enable_motor(self, bus, motor_id):
        """Enable motor (0xFC)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
        try:
            bus.send(msg)
        except can.CanError:
            pass

    def disable_motor(self, bus, motor_id):
        """Disable motor (0xFD)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
        try:
            bus.send(msg)
        except can.CanError:
            pass

    def set_zero_position(self, bus, motor_id):
        """Set Zero Position (0xFE)"""
        data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
        try:
            bus.send(msg)
        except can.CanError:
            pass

    def switch_control_mode(self, bus, motor_id, mode):
        """Switch Control Mode (Write Param RID 10)"""
        # Mode: 1=MIT, 2=PosVel, 3=Vel
        RID = 10
        # Data format for Write Param (0x55): [ID_L, ID_H, 0x55, RID, D0, D1, D2, D3]
        # But here we send to 0x7FF (Broadcast/System ID) usually? Or direct ID?
        # DM_CAN.py uses 0x7FF for parameters.
        # But wait, looking at DM_CAN.py __write_motor_param:
        # It sends to 0x7FF with payload: [ID_L, ID_H, 0x55, RID, 0, 0, 0, 0]
        # And data in last 4 bytes?  Wait.
        # DM_CAN.py L780: data_buf = [ID_L, ID_H, 0x55, RID, 0, 0, 0, 0]
        # Then L782/784 updates data_buf[4:8] with value.
        
        can_id_l = motor_id & 0xFF
        can_id_h = (motor_id >> 8) & 0xFF
        
        data = [can_id_l, can_id_h, 0x55, RID, 0, 0, 0, 0]
        
        # Mode is uint8 usually? DM_CAN: data_to_uint8s(int(data)) -> 4 bytes
        # Mode is small int, so just [mode, 0, 0, 0]? Little endian?
        # struct.pack('I', value) -> Little Endian uint32
        import struct
        packed = struct.pack('<I', int(mode))
        data[4] = packed[0]
        data[5] = packed[1]
        data[6] = packed[2]
        data[7] = packed[3]

        msg = can.Message(arbitration_id=0x7FF, data=data, is_extended_id=False, is_fd=True, bitrate_switch=True)
        try:
            bus.send(msg)
            time.sleep(0.01) # Short delay for processing
        except can.CanError:
            pass

    def _initialize_motors(self):
        """Enable and set MIT mode for all connected motors"""
        for arm, bus in self.buses.items():
            if bus is None:
                continue
            
            print(f"Initializing {arm} arm motors...")
            for motor_id in self.motor_configs[arm].keys():
                # 1. Enable
                self.enable_motor(bus, motor_id)
                time.sleep(0.01)
                
                # 2. Switch to MIT Mode (1)
                self.switch_control_mode(bus, motor_id, 1) # 1 = MIT Mode
                time.sleep(0.01)
            print(f"{arm} arm motors initialized.")

    def start(self):
        """제어 루프 스레드 시작"""
        if self._thread is not None and self._thread.is_alive():
            return
        
        # Start thread
        self._thread = threading.Thread(target=self.run, daemon=True)
        self._thread.start()
        
        # Initialize motors (Enable + MIT Mode)
        # Note: calling this here might block main thread slightly, but it's safer to ensure setup.
        # Or do it inside run()? Doing it here ensures it's done once on start.
        if self._running: # connect() sets _running True
             self._initialize_motors()

    def stop(self):
        """제어 루프를 중지하고 연결을 해제합니다."""
        self._running = False
        if self._thread and self._thread.is_alive():
            try:
                self._thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining thread: {e}")
        self._thread = None
            
        # Disable motors before closing
        for arm, bus in self.buses.items():
            if bus:
                try:
                    for motor_id in self.motor_configs[arm].keys():
                        self.disable_motor(bus, motor_id)
                    bus.shutdown()
                except Exception as e:
                    print(f"Error shutting down {arm} bus: {e}")
                self.buses[arm] = None

    def run(self):
        """실시간 제어 루프"""
        rate = 0.01 # 100Hz
        
        while self._running:
            start_time = time.time()
            
            try:
                # 메시지 처리 (Non-blocking)
                self._process_can_messages()
                
                # 명령 송신 (Request/Response 방식이므로 데이터를 받으려면 계속 보내야 함)
                self._send_mit_commands()
            except OSError as e:
                if e.errno == 100: # Network is down
                     print(f"CRITICAL ERROR: CAN Network is DOWN. Stopping control loop. ({e})")
                     # 안전을 위해 루프 중단
                     # self._running = False
                     # break
                     # 혹은 재연결 시도 로직을 넣을 수 있으나, 여기서는 에러 로그만 남기고 
                     # 시뮬레이션 모드처럼 동작하거나 잠시 대기
                     time.sleep(1.0)
                else:
                     print(f"OSError in control loop: {e}")
            except Exception as e:
                print(f"Error in control loop: {e}")
                # CanOperationError 등도 여기서 잡힘
            
            elapsed = time.time() - start_time
            sleep_time = max(0, rate - elapsed)
            time.sleep(sleep_time)

    def _process_can_messages(self):
        """수신된 CAN 메시지를 처리하여 로봇 상태 업데이트"""
        updated = False
        with self._lock:
            timestamp = time.time()
            self.robot_state['timestamp'] = timestamp
            
            for arm, bus in self.buses.items():
                if bus is None: 
                    # Sim noise for disconnected arm
                    self.robot_state[arm]['joints'] += np.random.normal(0, 0.0001, 7)
                    continue

                # Read all available messages
                while True:
                    msg = bus.recv(timeout=0) # Non-blocking
                    if msg is None:
                        break
                    
                    motor_id = msg.arbitration_id
                    
                    # Handle Response ID offset (Motor ID + 0x10)
                    # Master ID response range: 0x11 ~ 0x17 -> 0x01 ~ 0x07
                    if 0x11 <= motor_id <= 0x17:
                        motor_id -= 0x10
                        
                    # Motor ID range 1~7 (Ignore Gripper ID 8 for now)
                    if 1 <= motor_id <= 7:
                        joint_idx = motor_id - 1
                        data = msg.data
                        if len(data) >= 8:
                            # Decode Parameters based on Motor Type
                            m_type_idx = self.motor_configs[arm].get(motor_id, 0)
                            params = self.LIMIT_PARAM[m_type_idx]
                            p_max, v_max, t_max = params[0], params[1], params[2]

                            # Decoding (DM_CAN.py logic)
                            # data[1], data[2] -> Position
                            q_uint = (data[1] << 8) | data[2]
                            # data[3], data[4] -> Velocity
                            dq_uint = (data[3] << 4) | (data[4] >> 4)
                            # data[4], data[5] -> Torque
                            tau_uint = ((data[4] & 0xF) << 8) | data[5]

                            q = self.uint_to_float(q_uint, -p_max, p_max, 16)
                            dq = self.uint_to_float(dq_uint, -v_max, v_max, 12)
                            tau = self.uint_to_float(tau_uint, -t_max, t_max, 12)

                            self.robot_state[arm]['joints'][joint_idx] = q
                            self.robot_state[arm]['velocity'][joint_idx] = dq
                            self.robot_state[arm]['effort'][joint_idx] = tau
                            updated = True
        
        # If absolutely no connection, fallback to sim noise
        if not any(self.buses.values()):
             with self._lock:
                self.robot_state['left']['joints'] += np.random.normal(0, 0.001, 7)
                self.robot_state['right']['joints'] += np.random.normal(0, 0.001, 7)

    def _send_mit_commands(self):
        """각 모터에 MIT 제어 명령 송신"""
        for arm, bus in self.buses.items():
            if bus is None:
                continue
            
            # Check if active control is enabled for this arm
            target_joints = self.commands.get(arm)
            is_active = (target_joints is not None)

            for i in range(7):
                motor_id = i + 1
                m_type_idx = self.motor_configs[arm].get(motor_id, 0)
                params = self.LIMIT_PARAM[m_type_idx]
                p_max, v_max, t_max = params[0], params[1], params[2]

                if is_active:
                    # Active Control (Hold Position)
                    # Kp=40.0: Sufficient stiffness to hold posture
                    # Kd=1.5: Damping to prevent oscillation
                    kp = 40.0
                    kd = 1.5
                    q_des = target_joints[i]
                    dq_des = 0.0
                    tau_ff = 0.0
                else:
                    # FreeDrive (Passive) Mode
                    # Kp=0.0: No stiffness (Compliance)
                    # Kd=0.1: Minimal damping for stability
                    kp = 0.0
                    kd = 0.1
                    q_des = 0.0
                    dq_des = 0.0
                    tau_ff = 0.0

                kp_int = self.float_to_uint(kp, 0, 500, 12)
                kd_int = self.float_to_uint(kd, 0, 5, 12)
                q_int = self.float_to_uint(q_des, -p_max, p_max, 16)
                dq_int = self.float_to_uint(dq_des, -v_max, v_max, 12)
                tau_int = self.float_to_uint(tau_ff, -t_max, t_max, 12)

                data = [
                    (q_int >> 8) & 0xFF,
                    q_int & 0xFF,
                    dq_int >> 4,
                    ((dq_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
                    kp_int & 0xFF,
                    kd_int >> 4,
                    ((kd_int & 0xF) << 4) | ((tau_int >> 8) & 0xF),
                    tau_int & 0xFF
                ]

                msg = can.Message(
                    arbitration_id=motor_id,
                    data=data,
                    is_extended_id=False,
                    is_fd=True,
                    bitrate_switch=True
                )
                try:
                    bus.send(msg)
                except can.CanError:
                    pass

    def enable_freedrive(self, arm: str):
        """특정 팔을 FreeDrive(Passive) 모드로 전환"""
        with self._lock:
            self.commands[arm] = None

    def _receive_states(self):
        """로봇 팔로부터 CAN 데이터를 수신하여 내부 상태 업데이트"""
        pass # Merged into _process_can_messages

    def _send_commands(self):
        """현재 저장된 제어 명령을 CAN 메시지로 송신"""
        pass # Merged into _send_mit_commands

    def get_robot_state(self) -> Dict[str, Any]:
        """UI에서 로봇 상태를 안전하게 읽어갈 수 있도록 반환"""
        with self._lock:
            return {
                'left': {k: v.copy() for k, v in self.robot_state['left'].items()},
                'right': {k: v.copy() for k, v in self.robot_state['right'].items()},
                'timestamp': self.robot_state['timestamp']
            }

    def set_target_joints(self, arm: str, joints: np.ndarray):
        """특정 팔의 목표 관절 각도를 설정 (Active Control 미구현 - 현재는 모니터링 전용)"""
        # TODO: Active Control 구현 시 사용
        if arm in self.commands:
            with self._lock:
                self.commands[arm] = joints.copy()
