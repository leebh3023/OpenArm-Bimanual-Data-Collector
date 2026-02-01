# OpenArm Bimanual GUI 구현 결과 보고서

OpenArm Bimanual 로봇의 실시간 관제 및 데이터 수집을 위한 PyQt6 기반 맞춤형 GUI 제작을 완료했습니다. 요구하신 '단일 PC 구동' 및 'CAN 통신 기반 제어' 아키텍처가 모두 반영되었습니다.

## 🛠 구현된 주요 기능

1.  **실시간 CAN 제어 엔진**:
    - 별도 스레드에서 100Hz 이상의 주기로 로봇 상태를 수신하고 제어 명령을 송신합니다.
    - `python-can` 라이브러리를 통해 실제 CAN 버스와 연동됩니다. (하드웨어 미연결 시 시뮬레이션 모드 동작)
2.  **다중 카메라 스트리밍**:
    - OpenCV 스레딩을 활용하여 UI 주사율에 영향을 주지 않고 여러 대의 카메라 피드를 동시 출력합니다.
3.  **HDF5 데이터 수집 시스템**:
    - Task 이름을 지정하고 버튼 하나로 데이터 수집을 시작/종료할 수 있습니다.
    - 수집된 데이터는 `data/` 폴더에 `.h5` 형식으로 저장되며, VLA 모델(ACT, Diffusion Policy 등) 학습에 즉시 사용 가능한 구조를 가집니다.
    - **저장 데이터**: 타임스탬프, 양팔 관절 각도(Joint States), 그리고 모든 카메라의 영상 피드(RGB 이미지)를 하나의 파일에 포함합니다.
4.  **Intel RealSense D435 연동**:
    - `pyrealsense2` 라이브러리를 통해 카메라 영상을 안정적으로 스트리밍합니다.
    - `config/config.yaml` 파일을 통해 카메라의 Serial Number를 관리하며, USB 연결 포트가 바뀌어도 장치를 자동으로 인식합니다. (USB 3.2 SuperSpeed 권장)
5.  **다중 카메라 지원 (Tri-Camera)**:
    - **피라미드 레이아웃**: Top(중앙), Left/Right(하단) 배치를 통해 직관적인 모니터링 환경을 제공합니다.
    - **Semantic Data Logging**: HDF5 저장 시 `cam_0`, `cam_1` 대신 `top`, `left`, `right`와 같은 의미있는 이름을 사용하여 데이터셋을 관리합니다.
6.  **데이터 리플레이 (Data Replay)**:
    - 수집된 HDF5 데이터를 로드하여 로봇을 자동으로 움직일 수 있는 리플레이 기능을 탑재했습니다.
    - **Move-to-Start 안전 기능**: 리플레이 시작 전, 로봇이 현재 위에서 데이터의 시작 위치까지 3초간 부드럽게 이동하여 급격한 움직임으로 인한 사고를 예방합니다.
6.  **현대적인 UI 디자인**:
    - Dark Mode 디자인 시스템을 적용하여 가독성을 높였으며, 양팔 로봇의 대칭성을 고려한 레이아웃을 채택했습니다.

## 📂 프로젝트 구조

- `src/`
  - `core/can_controller.py`: CAN 통신 및 실시간 제어 로직
  - `ui/main_window.py`: 메인 GUI 윈도우 및 통합 관리
  - `widgets/`: 카메라 스트리밍, 데이터 수집, 리플레이 전용 위젯
- `config/`: 카메라 Serial Number 등 설정 파일 저장
- `data/`: 수집된 데이터 저장 디렉토리
- `doc/`: 프로젝트 문서
- `start_gui.sh`: GUI 실행 숏컷 스크립트
- `requirements.txt`: 필요한 Python 패키지 목록

## 🚀 설치 및 실행

로컬 시스템을 오염시키지 않도록 **가상 환경** 환경에서 실행하는 것을 권장합니다.

### 방법 1: `venv` 사용 (자동화)
제공된 실행 스크립트는 가상 환경이 없으면 자동으로 생성하고 의존성을 설치합니다.
```bash
cd openarm_gui
./start_gui.sh
```

### 방법 2: `conda` 사용 (권장 표준)
사용자 정의 표준 워크플로우에 따라 `conda` 환경을 수동으로 구축할 수도 있습니다.
```bash
# 환경 생성 및 활성화
conda create -n openarm python=3.10 -y
conda activate openarm

# 의존성 설치
cd openarm_gui
pip install -r requirements.txt

# 실행
./start_gui.sh
```

## ✅ 검증 결과
- **코드 무결성**: 모든 Python 소스 파일에 대한 문법 검사(`py_compile`)를 완료했으며 이상 없습니다.
- **멀티스레딩 안정성**: UI 스레드와 제어 스레드가 분리되어 동작함을 확인했습니다.
- **데이터 로깅**: 가상 상태 데이터를 기반으로 HDF5 파일이 정상 생성됨을 확인했습니다.

> [!TIP]
> 실물 CAN 장비 연결 시 `can_controller.py`의 `channel` 설정을 실제 사용하시는 장치(예: `can0`, `can1`)에 맞춰 변경하시기 바랍니다.
