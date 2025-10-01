#!/bin/bash

# 작업 시작 전 메시지 출력 함수
print_task_info() {
    echo "======================================================"
    echo "작업: $1"
    echo "시간: $(date +"%Y-%m-%d %H:%M:%S")"
    echo "설명: $2"
    echo "======================================================"
}

# 작업 로그 디렉토리 및 파일 생성
create_log_entry() {
    local task="$1"
    local description="$2"
    
    # 프로젝트 루트 디렉토리
    PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    LOG_DIR="${PROJECT_ROOT}/daily_log"
    TODAY=$(date +"%Y%m%d")
    LOG_FILE="${LOG_DIR}/${TODAY}.log"
    
    # 로그 디렉토리 생성 (없는 경우)
    if [ ! -d "$LOG_DIR" ]; then
        mkdir -p "$LOG_DIR"
    fi
    
    # 로그 파일 생성 (없는 경우)
    if [ ! -f "$LOG_FILE" ]; then
        cat > "$LOG_FILE" << EOF
# RoArm MCP 작업 로그
# 날짜: $(date +"%Y년 %m월 %d일")
# 작성자: RoArm M3

EOF
    fi
    
    # 로그 메시지 추가
    echo "[$(date +"%H:%M:%S")] $task: $description" >> "$LOG_FILE"
}

# 명령어를 실행하고 로그에 기록하는 함수
run_and_log() {
    local task="$1"
    local description="$2"
    local command="$3"
    
    print_task_info "$task" "$description"
    create_log_entry "$task" "$description"
    
    # 명령어 실행 전 표시
    echo "[실행] $command"
    
    # 명령어 실행 및 결과 저장
    eval "$command"
    local exit_code=$?
    
    # 실행 결과 로그에 기록
    if [ $exit_code -eq 0 ]; then
        create_log_entry "$task" "성공적으로 실행됨"
        echo "[결과] 성공"
    else
        create_log_entry "$task" "실행 실패 (코드: $exit_code)"
        echo "[결과] 실패 (코드: $exit_code)"
    fi
    
    return $exit_code
}

# 도움말 표시
if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    echo "사용법: $0 [명령]"
    echo ""
    echo "명령:"
    echo "  --help, -h            도움말 표시"
    echo "  --example             사용 예제 실행"
    echo ""
    echo "예제:"
    echo "  $0 --example"
    exit 0
fi

# 예제 실행
if [ "$1" == "--example" ]; then
    run_and_log "예제 실행" "로깅 시스템 테스트를 위한 예제 명령" "echo '예제 명령 실행 완료'"
    exit 0
fi

# 기본 메시지
if [ -z "$1" ]; then
    echo "RoArm MCP 작업 도구"
    echo "사용법에 대한 정보는 '$0 --help'를 실행하세요."
fi