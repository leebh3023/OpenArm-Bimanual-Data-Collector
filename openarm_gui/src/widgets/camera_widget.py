import cv2
import numpy as np
import pyrealsense2 as rs
from PyQt6.QtWidgets import QLabel, QSizePolicy
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QImage, QPixmap

class CameraThread(QThread):
    """카메라 영상을 획득하는 전용 스레드 (RealSense D435)"""
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def __init__(self, serial_number):
        super().__init__()
        self.serial_number = serial_number
        self._run_flag = True
        self.current_frame = None

    def run(self):
        if not self.serial_number:
            self._send_status_frame("No Serial Number")
            # Loop to keep thread alive but just sending status or sleeping
            while self._run_flag:
                self.msleep(1000)
            return

        pipeline = rs.pipeline()
        config = rs.config()
        
        try:
            print(f"Connecting to RealSense S/N: {self.serial_number}")
            config.enable_device(self.serial_number)
            # D435 standard: 640x480, 30fps. Using BGR8 for OpenCV compatibility.
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(config)
            print(f"RealSense S/N: {self.serial_number} started.")
        except Exception as e:
            print(f"Error starting RealSense {self.serial_number}: {e}")
            while self._run_flag:
                self._send_status_frame(f"Connect fail: {str(e)[:15]}...")
                self.msleep(2000)
            return

        while self._run_flag:
            try:
                # Wait for frames (blocking with timeout)
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                # Convert to numpy array (BGR)
                frame = np.asanyarray(color_frame.get_data())
                self.current_frame = frame
                self.change_pixmap_signal.emit(frame)
            except RuntimeError as e: # Timeout or device disconnected
                print(f"Frame error {self.serial_number}: {e}")
                self._send_status_frame("Frame Error / Timeout")
                # Try to reconnect or just wait? For now, just wait.
                self.msleep(1000)
            except Exception as e:
                print(f"Unexpected error {self.serial_number}: {e}")
                self.msleep(1000)

        pipeline.stop()
        print(f"RealSense S/N: {self.serial_number} stopped.")

    def _send_status_frame(self, message):
         # Make a black image with text
         dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
         cv2.putText(dummy_frame, message, (50, 240), 
                     cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
         self.change_pixmap_signal.emit(dummy_frame)

    def stop(self):
        self._run_flag = False
        self.wait()

class StreamingWidget(QLabel):
    """카메라 스트리밍을 표시하는 위젯"""
    def __init__(self, serial_number=""):
        super().__init__()
        self.serial_number = serial_number
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background-color: black; color: white; border: 1px solid #444;")
        
        display_text = f"Cam S/N: {serial_number}" if serial_number else "Cam Not Configured"
        self.setText(display_text)

        self.thread = CameraThread(serial_number)
        self.thread.change_pixmap_signal.connect(self.set_image)
    
    def start(self):
        self.thread.start()

    def stop(self):
        self.thread.stop()

    def get_current_frame(self):
        """현재 가장 최근의 카메라 프레임(BGR)을 반환"""
        return self.thread.current_frame

    def set_image(self, cv_img):
        """OpenCV 이미지(BGR)를 QPixmap으로 변환하여 표시"""
        # Display expects RGB for QImage
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        
        # 위젯 크기에 맞춰 리사이즈
        if self.width() > 0 and self.height() > 0:
            p = convert_to_Qt_format.scaled(self.width(), self.height(), Qt.AspectRatioMode.KeepAspectRatio)
            self.setPixmap(QPixmap.fromImage(p))
