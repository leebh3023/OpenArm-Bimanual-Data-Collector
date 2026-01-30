#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

# 가상 환경(venv) 관리 로직
VENV_DIR="${SCRIPT_DIR}/.venv"

if [ ! -d "${VENV_DIR}" ]; then
    echo "📦 가상 환경(.venv)이 존재하지 않습니다. 생성을 시작합니다..."
    python3 -m venv "${VENV_DIR}"
    source "${VENV_DIR}/bin/activate"
    pip install --upgrade pip
    pip install -r "${SCRIPT_DIR}/requirements.txt"
    echo "✅ 의존성 설치 완료."
else
    source "${VENV_DIR}/bin/activate"
fi

echo "🚀 OpenArm Bimanual Data Collector GUI를 시작합니다..."
python3 "${SCRIPT_DIR}/src/ui/main_window.py"
