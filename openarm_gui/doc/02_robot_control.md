# 02. 로봇 제어 시스템 (Robot Control System)

## 개요
로봇의 관절 제어 및 상태 수신은 `OpenArmCANController` 클래스가 담당합니다. 이 클래스는 `src/core/can_controller.py`에 정의되어 있으며, CAN 버스를 통해 로봇의 모터 드라이버와 통신합니다.

## 핵심 클래스: `OpenArmCANController`

### 역할
- **통신 관리**: `python-can` 라이브러리를 사용하여 CAN 인터페이스(`can0`, `can1` 등)를 엽니다.
- **실시간 루프**: 별도의 데몬 스레드에서 약 100Hz(10ms) 주기로 통신 루프를 실행합니다.
- **상태 동기화**: `robot_state` 딕셔너리에 최신 관절 각도, 속도, 토크 정보를 지속적으로 업데이트합니다.

### 주요 메서드
| 메서드 | 설명 |
| :--- | :--- |
| `connect()` | CAN 버스에 연결을 시도합니다. 실패 시 시뮬레이션 모드로 진입합니다. |
| `get_robot_state()` | Thread-safe하게 현재 로봇의 상태(Joints, Velocity 등)를 반환합니다. UI나 기록 스레드에서 이 메서드를 호출하여 데이터를 얻습니다. |
| `set_target_joints(arm, joints)` | 특정 팔('left'/'right')의 목표 관절 각도를 설정합니다. 실제 전송은 내부 루프에서 수행됩니다. |
| `run()` | 내부 스레드 루프입니다. `_receive_states()`로 데이터를 읽고, `_send_commands()`로 명령을 보냅니다. |

## 데이터 흐름
1.  **수신 (Read)**: `run()` 루프 내 `_receive_states()`가 CAN 버스에서 메시지를 읽어 `self.robot_state`를 갱신합니다.
2.  **접근 (Access)**: Main Window나 Data Collector는 `get_robot_state()`를 통해 최신 상태를 비동기적으로 가져갑니다.
3.  **명령 (Command)**: Replay 모듈 등이 `set_target_joints()`를 호출하면 `self.commands` 버퍼가 갱신됩니다.
4.  **송신 (Write)**: `run()` 루프 내 `_send_commands()`가 버퍼에 있는 목표치를 CAN 메시지로 변환하여 전송합니다.

## 주의사항
- **시뮬레이션 모드**: `python-can` 라이브러리가 없거나 연결 실패 시, 자동으로 시뮬레이션 모드(가짜 데이터 생성)로 동작하여 UI 개발을 용이하게 합니다.
- **Thread Safety**: `self._lock`을 사용하여 데이터 경쟁(Race Condition)을 방지합니다.
