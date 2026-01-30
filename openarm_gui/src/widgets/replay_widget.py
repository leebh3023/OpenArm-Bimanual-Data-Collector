import time
import os
import h5py
import numpy as np
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QLabel, QFileDialog, QProgressBar, QMessageBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal

class ReplayThread(QThread):
    """
    HDF5 데이터를 읽어 로봇 재생 명령을 전송하는 스레드.
    1. 현재 위치 -> 시작 위치 보간 (Interpolation)
    2. 데이터 재생 (Replay)
    """
    update_joints_signal = pyqtSignal(dict) # {'left': np.array, 'right': np.array}
    progress_signal = pyqtSignal(int, int)  # current_step, total_steps
    status_signal = pyqtSignal(str)         # Status message
    finished_signal = pyqtSignal()

    def __init__(self, filepath, current_state_callback):
        super().__init__()
        self.filepath = filepath
        self.get_current_state = current_state_callback
        self._running = False
        self._paused = False

    def run(self):
        self._running = True
        
        try:
            with h5py.File(self.filepath, 'r') as f:
                # 데이터 로드
                if 'observations' in f:
                    # New format
                    l_q = f['observations/left_q'][:]
                    r_q = f['observations/right_q'][:]
                else:
                    # Fallback or error
                    self.status_signal.emit("Error: Invalid HDF5 format")
                    return

                total_steps = len(l_q)
                target_l_0 = l_q[0]
                target_r_0 = r_q[0]

                # --- 1. 보간 (Move to Start) ---
                self.status_signal.emit("Moving to Start Position...")
                
                # 현재 로봇 상태 가져오기
                start_state = self.get_current_state()
                start_l = start_state['left']['joints']
                start_r = start_state['right']['joints']

                duration = 3.0 # seconds
                rate = 30 # Hz
                interp_steps = int(duration * rate)

                for step in range(interp_steps):
                    if not self._running: break
                    
                    alpha = (step + 1) / interp_steps
                    # Cosine interpolation for smoother start/stop
                    # alpha_smooth = (1 - np.cos(alpha * np.pi)) / 2 
                    # Linear is robust enough for now
                    alpha_smooth = alpha 

                    interp_l = start_l + (target_l_0 - start_l) * alpha_smooth
                    interp_r = start_r + (target_r_0 - start_r) * alpha_smooth

                    self.update_joints_signal.emit({
                        'left': interp_l,
                        'right': interp_r
                    })
                    time.sleep(1.0 / rate)

                if not self._running: return

                # --- 2. 재생 (Replay) ---
                self.status_signal.emit("Replaying...")
                for i in range(total_steps):
                    if not self._running: break
                    
                    # Pause handling could go here
                    
                    self.update_joints_signal.emit({
                        'left': l_q[i],
                        'right': r_q[i]
                    })
                    self.progress_signal.emit(i, total_steps)
                    time.sleep(1.0 / rate) # 30Hz fixed

            self.status_signal.emit("Replay Finished")
            self.finished_signal.emit()

        except Exception as e:
            self.status_signal.emit(f"Error: {str(e)}")
            print(f"Replay Error: {e}")

    def stop(self):
        self._running = False
        self.wait()


class ReplayWidget(QWidget):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.replay_thread = None
        self.filepath = ""
        
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 헤더
        header = QLabel("DATA REPLAY")
        header.setStyleSheet("color: #FFC107; font-weight: bold; margin-top: 10px;")
        layout.addWidget(header)
        
        # 파일 선택 영역
        file_layout = QHBoxLayout()
        self.file_label = QLabel("No file selected")
        self.file_label.setStyleSheet("color: #aaa; font-size: 10px;")
        self.file_label.setWordWrap(True)
        
        self.load_btn = QPushButton("LOAD")
        self.load_btn.setFixedWidth(60)
        self.load_btn.setStyleSheet("background-color: #555;")
        self.load_btn.clicked.connect(self._on_load_clicked)
        
        file_layout.addWidget(self.file_label)
        file_layout.addWidget(self.load_btn)
        layout.addLayout(file_layout)
        
        # 컨트롤 버튼
        self.play_btn = QPushButton("START REPLAY")
        self.play_btn.setFixedHeight(40)
        self.play_btn.setStyleSheet("background-color: #FFC107; color: black; font-weight: bold;")
        self.play_btn.setEnabled(False)
        self.play_btn.clicked.connect(self._on_play_clicked)
        layout.addWidget(self.play_btn)
        
        # 진행률바
        self.progress_bar = QProgressBar()
        self.progress_bar.setStyleSheet("QProgressBar { border: 0px; background: #333; height: 5px; } QProgressBar::chunk { background: #FFC107; }")
        self.progress_bar.setTextVisible(False)
        layout.addWidget(self.progress_bar)
        
        self.status_label = QLabel("Ready")
        self.status_label.setStyleSheet("color: #888; font-size: 10px;")
        layout.addWidget(self.status_label)

    def _on_load_clicked(self):
        # 데이터 디렉토리 기본값
        data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))
        fname, _ = QFileDialog.getOpenFileName(self, "Open HDF5 File", data_dir, "HDF5 Files (*.h5)")
        
        if fname:
            self.filepath = fname
            self.file_label.setText(os.path.basename(fname))
            self.play_btn.setEnabled(True)
            self.status_label.setText("File loaded.")

    def _on_play_clicked(self):
        if self.replay_thread and self.replay_thread.isRunning():
            # Stop
            self.replay_thread.stop()
            self.play_btn.setText("START REPLAY")
            self.play_btn.setStyleSheet("background-color: #FFC107; color: black; font-weight: bold;")
            self.status_label.setText("Stopped.")
        else:
            # Start
            if not self.filepath: return
            
            # 콜백: 스레드가 시작될 때의 최신 로봇 상태를 가져오기 위함
            self.replay_thread = ReplayThread(self.filepath, self.controller.get_robot_state)
            self.replay_thread.update_joints_signal.connect(self._send_command)
            self.replay_thread.progress_signal.connect(self._update_progress)
            self.replay_thread.status_signal.connect(self._update_status)
            self.replay_thread.finished_signal.connect(self._on_finished)
            
            self.replay_thread.start()
            
            self.play_btn.setText("STOP")
            self.play_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")

    def _send_command(self, targets):
        self.controller.set_target_joints('left', targets['left'])
        self.controller.set_target_joints('right', targets['right'])

    def _update_progress(self, current, total):
        self.progress_bar.setMaximum(total)
        self.progress_bar.setValue(current)

    def _update_status(self, msg):
        self.status_label.setText(msg)

    def _on_finished(self):
        self.play_btn.setText("START REPLAY")
        self.play_btn.setStyleSheet("background-color: #FFC107; color: black; font-weight: bold;")
        self.progress_bar.setValue(0)
