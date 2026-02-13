#!/usr/bin/env bash
# openarm_gui 관련 스크립트를 .venv 파이썬으로 실행하는 헬퍼 스크립트

set -e

# 스크립트 위치 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PYTHON="${SCRIPT_DIR}/.venv/bin/python"

# .venv 존재 확인
if [ ! -f "${VENV_PYTHON}" ]; then
    echo "❌ 에러: .venv 가상 환경이 존재하지 않습니다. 'start_gui.sh'를 먼저 실행하여 환경을 구축하세요."
    exit 1
fi

# PATH 및 PYTHONPATH 설정 (가상 환경 bin 및 프로젝트 루트 추가)
export PATH="${SCRIPT_DIR}/.venv/bin:${PATH}"
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

# 인자로 받은 스크립트 실행
if [ $# -eq 0 ]; then
    echo "사용법: $0 [파이썬_스크립트_경로] [추가_인자...]"
    exit 1
fi

echo "🚀 .venv 환경에서 실행 중: $@"
"${VENV_PYTHON}" "$@"
