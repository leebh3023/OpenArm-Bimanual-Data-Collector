import cv2
import numpy as np
from PyQt6.QtWidgets import QLabel, QSizePolicy
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap

class CameraThread(QThread):
    """카메라 영상을 획득하는 전용 스레드"""
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self._run_flag = True
        self.current_frame = None

    def run(self):
        cap = cv2.VideoCapture(self.camera_id)
        while self._run_flag:
            ret, frame = cap.read()
            if ret:
                self.current_frame = frame
                self.change_pixmap_signal.emit(frame)
            else:
                # 카메라 연결 실패 시 시뮬레이션용 빈 이미지 생성
                dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(dummy_frame, f"Cam {self.camera_id} Disconnected", (150, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                self.change_pixmap_signal.emit(dummy_frame)
                self.msleep(500)
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class StreamingWidget(QLabel):
    """카메라 스트리밍을 표시하는 위젯"""
    def __init__(self, camera_id=0):
        super().__init__()
        self.camera_id = camera_id
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background-color: black; color: white; border: 1px solid #444;")
        self.setText(f"Camera {camera_id} Loading...")

        self.thread = CameraThread(camera_id)
        self.thread.change_pixmap_signal.connect(self.set_image)
    
    def start(self):
        self.thread.start()

    def stop(self):
        self.thread.stop()

    def get_current_frame(self):
        """현재 가장 최근의 카메라 프레임을 반환"""
        return self.thread.current_frame

    def set_image(self, cv_img):
        """OpenCV 이미지를 QPixmap으로 변환하여 표시"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        
        # 위젯 크기에 맞춰 리사이즈
        p = convert_to_Qt_format.scaled(self.width(), self.height(), Qt.AspectRatioMode.KeepAspectRatio)
        self.setPixmap(QPixmap.fromImage(p))
