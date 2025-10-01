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

# 가상환경 활성화 체크 및 활성화
if [[ -z "$VIRTUAL_ENV" ]]; then
    echo "[작업] 가상환경 활성화 중..."
    source "$VENV_DIR/bin/activate"
    log_message "가상환경 활성화됨"
fi

# MCP 서버 실행
echo "[작업] MCP 서버 시작 중..."
cd "$PROJECT_ROOT"
log_message "서버 시작 명령 실행: python -m mcp_server.server"
python -m mcp_server.server

# 종료 시 로그 추가
log_message "서버 종료"