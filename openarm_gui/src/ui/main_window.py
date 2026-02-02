import sys
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import time
import sys
import os
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
import time
import h5py
import cv2
import yaml
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QFrame, QSplitter, QStatusBar, QMessageBox, QSizePolicy, QGridLayout)
from PyQt6.QtCore import Qt, QTimer, pyqtSlot, QThread, pyqtSignal
from PyQt6.QtGui import QImage, QPixmap, QAction, QFont, QIcon

# 로컬 임포트 (패키지 구조에 맞게 조정)
try:
    from openarm_gui.src.core.can_controller import OpenArmCANController
    from openarm_gui.src.widgets.camera_widget import StreamingWidget
    from openarm_gui.src.widgets.collection_widget import CollectionWidget
    from openarm_gui.src.widgets.replay_widget import ReplayWidget
except ImportError:
    # 직접 실행 시를 위한 경로 추가
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    from src.core.can_controller import OpenArmCANController
    from src.widgets.camera_widget import StreamingWidget
    from src.widgets.collection_widget import CollectionWidget
    from src.widgets.camera_widget import StreamingWidget
    from src.widgets.collection_widget import CollectionWidget
    from src.widgets.replay_widget import ReplayWidget

class GoToZeroThread(QThread):
    """
    현재 위치에서 0점 위치(모든 관절 0도)로 부드럽게 이동하는 스레드.
    안전을 위해 5초 동안 천천히 보간합니다.
    """
    update_joints_signal = pyqtSignal(dict) # {'left': np.array, 'right': np.array}
    status_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()

    def __init__(self, current_state_callback):
        super().__init__()
        self.get_current_state = current_state_callback
        self._running = False

    def run(self):
        self._running = True
        self.status_signal.emit("Moving to Zero Position...")
        
        try:
            # 현재 상태
            start_state = self.get_current_state()
            start_l = np.array(start_state['left']['joints'])
            start_r = np.array(start_state['right']['joints'])
            
            # 목표 상태 (0점)
            target_l = np.zeros(7)
            target_r = np.zeros(7)
            
            duration = 5.0 # 5초 (안전 우선)
            rate = 30 # Hz
            steps = int(duration * rate)
            
            for step in range(steps):
                if not self._running: break
                
                # S자 곡선 (Cosine) 보간으로 부드러운 가감속
                alpha = (step + 1) / steps
                alpha_smooth = (1 - np.cos(alpha * np.pi)) / 2
                
                curr_l = start_l + (target_l - start_l) * alpha_smooth
                curr_r = start_r + (target_r - start_r) * alpha_smooth
                
                self.update_joints_signal.emit({
                    'left': curr_l,
                    'right': curr_r
                })
                time.sleep(1.0 / rate)
                
            if self._running:
                self.status_signal.emit("Reached Zero Position.")
                
        except Exception as e:
            self.status_signal.emit(f"Error: {e}")
            
        self.finished_signal.emit()

    def stop(self):
        self._running = False
        self.wait()

class OpenArmMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenArm Bimanual Data Collector")
        self.resize(1400, 900)    # <--- 여기서 가로 1400, 세로 900으로 정의함
        
        # 데이터 저장 관련
        self.recording = False
        self.current_task = ""
        self.log_file = None # h5py.File object
        
        # CAN 컨트롤러 초기화
        self.controller = OpenArmCANController()
        
        # UI 구성
        self._setup_ui()
        
        # 센서 데이터 업데이트 타이머 (30Hz)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_all)
        self.update_timer.start(33) 

    def _setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # 1. 좌측 제어 패널
        control_panel = QFrame()
        control_panel.setFixedWidth(320)
        control_panel.setStyleSheet("background-color: #2b2b2b; border-radius: 8px; border: 1px solid #444;")
        control_layout = QVBoxLayout(control_panel)
        
        header = QLabel("OPENARM CONTROL")
        header.setStyleSheet("color: #4CAF50; font-size: 18px; font-weight: bold; border: none;")
        control_layout.addWidget(header)
        
        # 연결 버튼
        self.conn_btn = QPushButton("CONNECT ROBOT")
        self.conn_btn.setFixedHeight(50)
        self.conn_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self.conn_btn.clicked.connect(self._on_connect_clicked)
        self.conn_btn.clicked.connect(self._on_connect_clicked)
        control_layout.addWidget(self.conn_btn)

        # 영점 이동 버튼
        self.zero_btn = QPushButton("GO TO ZERO")
        self.zero_btn.setFixedHeight(40)
        self.zero_btn.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold;")
        self.zero_btn.clicked.connect(self._on_zero_clicked)
        self.zero_btn.setEnabled(False) # 연결 전에는 비활성
        control_layout.addWidget(self.zero_btn)
        
        control_layout.addSpacing(20)
        
        # 데이터 수집 위젯
        self.collection_widget = CollectionWidget()
        self.collection_widget.recording_toggled.connect(self._on_recording_toggled)
        self.collection_widget.task_started.connect(self._on_task_started)
        control_layout.addWidget(self.collection_widget)

        control_layout.addSpacing(20)

        # 리플레이 위젯
        self.replay_widget = ReplayWidget(self.controller)
        control_layout.addWidget(self.replay_widget)
        
        control_layout.addStretch()
        
        # 2. 우측 모니터링 패널
        monitor_panel = QWidget()
        monitor_layout = QVBoxLayout(monitor_panel)
        
        # 카메라 뷰포트 (실제 위젯 사용)
        # 설정 파일 로드
        self.config = self._load_config()

        # 카메라 뷰포트 (실제 위젯 사용)
        # Camera Setup (Pyramid Layout)
        self.camera_grid = QGridLayout()
        self.cameras = {} # Dictionary for semantic access: 'top', 'left', 'right'

        # Config keys
        cam_configs = {
            'top':   {'serial': self.config.get('cameras', {}).get('top_camera_serial', ''),   'pos': (0, 0, 1, 2)}, # Row 0, Col 0, RowSpan 1, ColSpan 2
            'left':  {'serial': self.config.get('cameras', {}).get('left_camera_serial', ''),  'pos': (1, 0, 1, 1)}, # Row 1, Col 0
            'right': {'serial': self.config.get('cameras', {}).get('right_camera_serial', ''), 'pos': (1, 1, 1, 1)}  # Row 1, Col 1
        }

        for name, cfg in cam_configs.items():
            serial = cfg['serial']
            cam_widget = StreamingWidget(serial_number=serial)
            
            # Grid Layout: addWidget(widget, row, col, rowSpan, colSpan)
            r, c, rs, cs = cfg['pos']
            self.camera_grid.addWidget(cam_widget, r, c, rs, cs)
            
            self.cameras[name] = cam_widget
            
            if serial:
                cam_widget.start()

        
        monitor_layout.addLayout(self.camera_grid, stretch=3)
        
        # 로봇 상태 정보
        status_panel = QFrame()
        status_panel.setFixedHeight(200)
        status_panel.setStyleSheet("background-color: #1a1a1a; border-radius: 8px;")
        status_layout = QHBoxLayout(status_panel)
        
        self.left_status = QLabel("Left Arm: Idle")
        self.right_status = QLabel("Right Arm: Idle")
        self.left_status.setStyleSheet("color: #00e5ff; font-family: monospace; font-size: 14px;")
        self.right_status.setStyleSheet("color: #ff4081; font-family: monospace; font-size: 14px;")
        
        # 텍스트가 길어져도 레이아웃이 터지지 않게 설정
        self.left_status.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        self.right_status.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        
        status_layout.addWidget(self.left_status)
        status_layout.addWidget(self.right_status)
        monitor_layout.addWidget(status_panel, stretch=1)
        
        main_layout.addWidget(control_panel)
        main_layout.addWidget(monitor_panel)
        
        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("Ready")

    def _on_connect_clicked(self):
        if not self.controller._running:
            if self.controller.connect():
                self.controller.start()
                self.conn_btn.setText("DISCONNECT")
                self.conn_btn.setStyleSheet("background-color: #f44336; color: white;")
                self.zero_btn.setEnabled(True) # 연결됨 -> 활성화
                self.statusBar().showMessage("Connected via CAN")
        else:
            self.controller.stop()
            self.conn_btn.setText("CONNECT ROBOT")
            self.conn_btn.setStyleSheet("background-color: #4CAF50; color: white;")
            self.zero_btn.setEnabled(False) # 비활성화
            self.statusBar().showMessage("Disconnected")

        # self.connection_widget.connection_changed.emit(False)
        # self.connection_widget.gripper_changed.emit(False)
    
    def _on_zero_clicked(self):
        # 영점 이동 시작
        self.zero_btn.setEnabled(False)
        self.conn_btn.setEnabled(False)
        self.collection_widget.setEnabled(False)
        self.replay_widget.setEnabled(False)
        
        self.zero_thread = GoToZeroThread(self.controller.get_robot_state)
        self.zero_thread.update_joints_signal.connect(self._send_command)
        self.zero_thread.status_signal.connect(self.statusBar().showMessage)
        self.zero_thread.finished_signal.connect(self._on_zero_finished)
        self.zero_thread.start()

    def _send_command(self, targets):
        self.controller.set_target_joints('left', targets['left'])
        self.controller.set_target_joints('right', targets['right'])

    def _on_zero_finished(self):
        # 종료 후 버튼 복구
        self.zero_btn.setEnabled(True)
        self.conn_btn.setEnabled(True)
        self.collection_widget.setEnabled(True)
        self.replay_widget.setEnabled(True)
        self.statusBar().showMessage("Ready")

    def _on_task_started(self, task_name):
        self.current_task = task_name

    def _on_recording_toggled(self, is_start):
        self.recording = is_start
        if is_start:
            # 파일 생성
            data_dir = os.path.join(os.path.dirname(__file__), "..", "..", "data")
            os.makedirs(data_dir, exist_ok=True)
            filename = f"{self.current_task}_{int(time.time())}.h5"
            self.log_path = os.path.join(data_dir, filename)
            
            # HDF5 파일 초기화
            self.log_file = h5py.File(self.log_path, 'w')
            # VLA 표준 그룹 구조 (예: ACT/Diffusion)
            self.obs_group = self.log_file.create_group('observations')
            self.images_group = self.obs_group.create_group('images')
            
            # 동적 크기 할당을 위한 데이터셋 생성 (chunked)
            self.ts_dataset = self.obs_group.create_dataset('timestamp', (0,), maxshape=(None,), dtype='f8', chunks=True)
            self.l_q_dataset = self.obs_group.create_dataset('left_q', (0, 7), maxshape=(None, 7), dtype='f4', chunks=True)
            self.r_q_dataset = self.obs_group.create_dataset('right_q', (0, 7), maxshape=(None, 7), dtype='f4', chunks=True)
            
            # 카메라별 데이터셋 (Semantic Names)
            self.cam_datasets = {}
            for name, cam in self.cameras.items(): # name: 'top', 'left', 'right'
                # Path: observations/images/top, ...
                self.cam_datasets[name] = self.images_group.create_dataset(
                    name, (0, 480, 640, 3), maxshape=(None, 480, 640, 3), dtype='u1', chunks=(1, 480, 640, 3)
                )

            self.statusBar().showMessage(f"Recording to {filename}...")
        else:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.statusBar().showMessage("Recording saved (HDF5).")

    def _update_all(self):
        state = self.controller.get_robot_state()
        
        # 상태 텍스트 업데이트
        l_joints = ", ".join([f"{j:5.2f}" for j in state['left']['joints']])
        r_joints = ", ".join([f"{j:5.2f}" for j in state['right']['joints']])
        self.left_status.setText(f"LEFT ARM JOINTS:\n[{l_joints}]")
        self.right_status.setText(f"RIGHT ARM JOINTS:\n[{r_joints}]")

        # 데이터 로깅 (HDF5)
        if self.recording and self.log_file:
            curr_idx = self.ts_dataset.shape[0]
            new_shape = (curr_idx + 1,)
            
            # 1. 텍스트/수치 데이터 확장 및 저장
            self.ts_dataset.resize(new_shape)
            self.ts_dataset[curr_idx] = state['timestamp']
            
            self.l_q_dataset.resize((curr_idx + 1, 7))
            self.l_q_dataset[curr_idx] = state['left']['joints']
            
            self.r_q_dataset.resize((curr_idx + 1, 7))
            self.r_q_dataset[curr_idx] = state['right']['joints']

            # 2. 이미지 데이터 확장 및 저장
            for name, cam in self.cameras.items():
                frame = cam.get_current_frame()
                if frame is not None:
                    # cam_datasets keys are 'top', 'left', 'right'
                    ds = self.cam_datasets[name]
                    ds.resize((curr_idx + 1, 480, 640, 3))
                    # RGB 변환 (OpenCV BGR -> RGB)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    # 리사이즈 확인 (480, 640 고정 가정)
                    if frame_rgb.shape[:2] != (480, 640):
                        frame_rgb = cv2.resize(frame_rgb, (640, 480))
                    ds[curr_idx] = frame_rgb

    def _load_config(self):
        try:
            import yaml
            config_path = os.path.join(os.path.dirname(__file__), "..", "..", "config", "config.yaml")
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            print(f"Config load failed: {e}")
            return {}

    def closeEvent(self, event):
        """종료 시 안전하게 정리"""
        self.controller.stop()
        for cam in self.cameras.values():
            cam.stop()
        if self.log_file:
            self.log_file.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    # 어두운 테마 팔레트 설정
    from PyQt6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.ColorGroup.All, QPalette.ColorRole.Window, QColor(40, 40, 40))
    palette.setColor(QPalette.ColorGroup.All, QPalette.ColorRole.WindowText, Qt.GlobalColor.white)
    app.setPalette(palette)
    
    window = OpenArmMainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
