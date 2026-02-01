# 04. 리플레이 및 안전 기능 (Replay & Safety)

## 개요
수집된 데이터를 로봇이 그대로 따라 하도록 하거나(Replay), 안전하게 초기 위치로 복귀(Go to Zero)하는 기능에 대한 설명입니다. **안전(Safety)**이 최우선 고려 사항입니다.

## 1. 리플레이 시스템 (`src/widgets/replay_widget.py`)

### `ReplayThread`
리플레이는 단순한 데이터 전송이 아니라 **두 단계**로 이루어집니다.

1.  **보간 이동 (Move-to-Start Interpolation)**
    - 리플레이 시작 시, 로봇의 현재 위치와 데이터 파일의 첫 프레임(Start Frame)이 다를 수 있습니다.
    - 갑작스러운 점프(Jump)를 막기 위해, 3초간 현재 위치에서 시작 위치까지 부드럽게 이동하는 궤적을 실시간으로 생성하여 전송합니다.
    
2.  **데이터 재생 (Playback)**
    - 보간이 끝나면 HDF5 파일의 `observations/left_q`, `observations/right_q` 데이터를 읽어 30Hz 주기로 컨트롤러에 전송합니다.

### 데이터 포맷 (HDF5)
리플레이 파일은 다음 구조를 따라야 합니다:
```
/observations
    /left_q  [N, 7]  # 왼쪽 팔 관절 각도
    /right_q [N, 7]  # 오른쪽 팔 관절 각도
```

## 2. 영점 이동 기능 (Go To Zero)

### `GoToZeroThread` (`src/ui/main_window.py`)
로봇을 안전하게 초기 포즈(모든 관절 0도)로 복귀시키는 기능입니다.

- **동작 원리**:
    - 현재 로봇의 관절 각도를 읽어옵니다.
    - 목표 위치(0도)까지 **5초** 동안 이동하는 궤적을 생성합니다.
    - S-Curve(Cosine) 또는 선형 보간을 사용하여 시작과 끝이 부드럽습니다.
- **안전 장치**:
    - 이동 중에는 다른 제어 버튼(연결 해제, 리플레이 등)이 비활성화됩니다.
    - 5초라는 충분한 시간을 두어 충돌 위험을 최소화합니다.

## 코드 예시 (Interpolation Logic)
```python
# 현재 위치에서 목표 위치까지 'steps'만큼 나누어 이동
alpha = (step + 1) / steps
current_pos = start_pos + (target_pos - start_pos) * alpha
controller.set_target_joints(current_pos)
```
