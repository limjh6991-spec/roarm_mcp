#!/bin/bash

# 프로젝트 루트 디렉토리
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_ROOT}/.venv"
LOG_DIR="${PROJECT_ROOT}/daily_log"
TODAY=$(date +"%Y%m%d")
LOG_FILE="${LOG_DIR}/${TODAY}.log"

# 로그 디렉토리 생성 (없는 경우)
if [ ! -d "$LOG_DIR" ]; then
    echo "[작업] 로그 디렉토리 생성: $LOG_DIR"
    mkdir -p "$LOG_DIR"
fi

# 로그 파일 생성 (없는 경우)
if [ ! -f "$LOG_FILE" ]; then
    echo "[작업] 새 로그 파일 생성: $LOG_FILE"
    cat > "$LOG_FILE" << EOF
# RoArm MCP 작업 로그
# 날짜: $(date +"%Y년 %m월 %d일")
# 작성자: RoArm M3

EOF
fi

# 로그 메시지 추가
log_message() {
    local message="$1"
    local timestamp=$(date +"%H:%M:%S")
    echo "[$(date +"%Y-%m-%d %H:%M:%S")] $message" >> "$LOG_FILE"
    echo "[로그] $message"
}

echo "[작업] === RoArm MCP 환경 설정 시작 ==="
log_message "환경 설정 시작"

# 가상환경 생성
if [ ! -d "$VENV_DIR" ]; then
    echo "[작업] Python 가상환경 생성 중..."
    python3 -m venv "$VENV_DIR"
    log_message "가상환경 생성됨: $VENV_DIR"
    echo "[작업] 가상환경이 생성되었습니다: $VENV_DIR"
else
    echo "[작업] 가상환경이 이미 존재합니다: $VENV_DIR"
    log_message "기존 가상환경 사용: $VENV_DIR"
fi

# 가상환경 활성화
echo "[작업] 가상환경 활성화 중..."
source "$VENV_DIR/bin/activate"
log_message "가상환경 활성화됨"

# 필요한 패키지 설치
echo "[작업] 필수 패키지 설치 중..."
log_message "pip 및 기본 도구 업그레이드 시작"
pip install --upgrade pip setuptools wheel
log_message "프로젝트 의존성 설치 시작"
pip install -r "${PROJECT_ROOT}/requirements.txt"
log_message "패키지 설치 완료"

# dialout 그룹 확인 (하드웨어 통신을 위해)
if ! groups | grep -q "dialout"; then
    echo "[작업] 경고: 현재 사용자가 dialout 그룹에 속해있지 않습니다."
    echo "[작업] 하드웨어 통신을 위해 다음 명령을 실행하세요:"
    echo "[작업] sudo usermod -a -G dialout $USER"
    echo "[작업] 명령 실행 후 재로그인이 필요합니다."
    log_message "경고: dialout 그룹 권한 없음 - 하드웨어 통신에 필요"
fi

echo ""
echo "[작업] === RoArm MCP 환경 설정 완료 ==="
log_message "환경 설정 완료"
echo "[작업] 가상환경을 활성화하려면: source ${VENV_DIR}/bin/activate"
echo "[작업] MCP 서버를 실행하려면: bash ${PROJECT_ROOT}/scripts/run_mcp_server.sh"