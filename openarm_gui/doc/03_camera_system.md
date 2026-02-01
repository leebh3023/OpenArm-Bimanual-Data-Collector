# 03. 카메라 시스템 (Camera System)

## 개요
Intel RealSense D435 카메라의 영상을 스트리밍하고 UI에 표시합니다. 여러 대의 카메라(Left, Right, Top)를 동시에 지원하기 위해 시리얼 번호(S/N) 기반으로 장치를 식별합니다.

## 핵심 구성 요소

### 1. `CameraThread` (`src/widgets/camera_widget.py`)
카메라 한 대당 하나의 `CameraThread`가 생성됩니다.
- **초기화**: `pyrealsense2` 파이프라인을 생성하고 지정된 시리얼 번호의 장치를 엽니다.
- **스트리밍**: BGR8 포맷, 640x480 해상도, 30FPS로 설정을 고정하여 대역폭을 관리합니다.
- **신호 방출**: 새로운 프레임이 도착할 때마다 `change_pixmap_signal`을 방출하여 UI로 보냅니다.
- **에러 처리**: 연결 해제나 타임아웃 발생 시, 검은 화면에 에러 메시지를 띄우고 재연결을 대기합니다.

### 2. `StreamingWidget`
- `QLabel`을 상속받아 이미지를 표시하는 UI 위젯입니다.
- OpenCV 포맷(BGR)의 이미지를 받아 Qt 포맷(RGB)으로 변환하고, 위젯 크기에 맞게 리사이즈(`Scaled`)하여 보여줍니다.

## 설정 관리 (`config/config.yaml`)
카메라의 물리적 위치(Top, Left, Right)와 시리얼 번호 매핑은 설정 파일에서 관리합니다.
```yaml
camera:
  top: "00000000"
  left: "11111111"
  right: "22222222"
```
앱 실행 시 이 파일을 로드하여 `CameraWidget`들을 동적으로 생성/배치합니다.

## 성능 최적화
- **QThread 사용**: 이미지 디코딩 및 수신 대기(`wait_for_frames`)와 같은 블로킹 작업이 UI를 멈추게 하지 않도록 별도 스레드에서 처리합니다.
- **해상도 제한**: USB 대역폭 포화 방지를 위해 불필요하게 높은 해상도는 사용하지 않습니다.
