# OpenArm Bimanual Data Collector

## 🏫 기관 정보
- **KONKUK univ RVLAB 연구실**
- **제작자**: 이병현 (Lee Byeong-hyeon)
- **이메일**: [leebh3023@gmail.com](mailto:leebh3023@gmail.com)

## 📝 프로젝트 개요
이 프로젝트는 OpenArm Bimanual 로봇의 실시간 관제 및 학습 데이터 수집을 위한 전용 GUI 시스템입니다. 
- **실시간 모니터링**: 듀얼 암 관절 상태 및 다중 카메라 스트리밍 지원
- **데이터 저장**: VLA 모델(ACT, Diffusion Policy 등) 학습에 최적화된 **HDF5(.h5)** 형식 저장 (관절 상태 + 영상 데이터 통합)
- **안정적 제어**: CAN 통신 기반의 독립된 고성능 제어 루프

## 🚀 실행 방법

### 1. 사전 준비
로컬 시스템 환경 오염을 방지하기 위해 가상 환경 사용을 강력히 권장합니다.

### 2. GUI 실행 (자동화)
제공된 `start_gui.sh` 스크립트를 실행하면 가상 환경 생성부터 의존성 설치, 프로그램 실행까지 자동으로 진행됩니다.
```bash
cd openarm_gui
./start_gui.sh
```

### 3. 상세 가이드
더 자세한 기술 스펙 및 사용법은 [doc/Walkthrough.md](openarm_gui/doc/Walkthrough.md) 파일을 참고해 주세요.
