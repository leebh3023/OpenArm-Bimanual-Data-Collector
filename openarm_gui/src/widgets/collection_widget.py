from PyQt6.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit, QHBoxLayout
from PyQt6.QtCore import pyqtSignal

class CollectionWidget(QWidget):
    """데이터 수집 및 Task 제어를 위한 위젯"""
    task_started = pyqtSignal(str) # Task 이름
    recording_toggled = pyqtSignal(bool) # True: Start, False: Stop

    def __init__(self):
        super().__init__()
        self.is_recording = False
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Task Name 입력
        task_layout = QHBoxLayout()
        task_layout.addWidget(QLabel("Task Name:"))
        self.task_input = QLineEdit("task_001")
        task_layout.addWidget(self.task_input)
        layout.addLayout(task_layout)

        # 수집 버튼
        self.record_btn = QPushButton("START RECORDING")
        self.record_btn.setFixedHeight(50)
        self.record_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        self.record_btn.clicked.connect(self._toggle_recording)
        layout.addWidget(self.record_btn)

        # 상태 표시
        self.status_label = QLabel("Status: Idle")
        layout.addWidget(self.status_label)

    def _toggle_recording(self):
        self.is_recording = not self.is_recording
        if self.is_recording:
            self.task_started.emit(self.task_input.text())
            self.recording_toggled.emit(True)
            self.record_btn.setText("STOP RECORDING")
            self.status_label.setText("Status: Recording...")
            self.task_input.setEnabled(False)
        else:
            self.recording_toggled.emit(False)
            self.record_btn.setText("START RECORDING")
            self.status_label.setText("Status: Idle")
            self.task_input.setEnabled(True)
