# 🤖 RoArm MCP 프로젝트 작업 수행 가이드
> **AI 에이전트 전용 RoArm MCP 프로젝트 작업 지침서**  
> 최종 업데이트: 2025-10-02  
> 용도: RoArm MCP 프로젝트 모든 작업 시작 전 필독 및 준수 사항

---

## 🎯 **작업 시작 전 필수 확인사항**

### **1. 환경 준비 체크리스트 (매번 필수)**
```bash
# 1) 올바른 디렉토리 확인
cd /home/roarm_m3/dev_roarm/roarm_mcp
pwd  # 경로 확인

# 2) Python 환경 설정 확인
python --version  # Python 3.8+ 확인
which python      # Python 경로 확인

# 3) Isaac Sim 환경 확인 (필요시)
ls -la ~/isaac_sim/  # Isaac Sim 설치 확인
echo "Isaac Sim 경로: ~/isaac_sim"

# 4) Git 상태 확인
git status  # 현재 변경사항 확인
git branch  # 현재 브랜치 확인 (main)
```

### **⚠️ Isaac Sim 및 MCP 환경 관리 핵심 규칙**

**🚫 절대 금지 사항**:
- Isaac Sim 환경 설정 없이 시뮬레이션 스크립트 실행
- MCP 서버 실행 중인 터미널에서 다른 명령어 실행 (서버 종료 위험)
- 로봇 제어 스크립트를 안전 확인 없이 실행
- backup 디렉토리 파일을 Git에 커밋

**✅ 필수 실행 규칙**:
- 모든 Isaac Sim 관련 작업은 Isaac Sim 환경에서 실행
- MCP 서버와 클라이언트는 별도 터미널에서 실행
- 로봇 제어 전 시뮬레이션 환경에서 안전성 검증
- 매 실행 전 requirements.txt 의존성 확인

**🛡️ 서버-클라이언트 분리 전략**:
- **MCP 서버**: 전용 터미널 또는 백그라운드 실행
- **MCP 클라이언트**: 별도 터미널에서 테스트
- **Isaac Sim**: 독립적인 환경에서 실행
- **상태 확인**: `ps aux | grep python` 로 프로세스 생존 여부 확인

**올바른 실행 예시 (2025-10-02 업데이트):**
```bash
# ✅ Isaac Sim 5.0 MCP 서버 실행 (필수: Isaac Sim 환경에서 실행)
cd /home/roarm_m3/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless

# ✅ 백그라운드 실행 (서버 운영용)
cd /home/roarm_m3/isaac_sim
nohup ./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless > /home/roarm_m3/dev_roarm/roarm_mcp/logs/isaac_sim_mcp.log 2>&1 &

# ✅ 클라이언트 테스트 (별도 터미널)
cd /home/roarm_m3/dev_roarm/roarm_mcp
python examples/isaac_sim_client.py --server-url ws://localhost:8765 --robot-type ur10

# ✅ 완성된 Isaac Sim PhysX Tensors 솔루션 단독 실행
cd /home/roarm_m3/isaac_sim
./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py

# ❌ 잘못된 방법들
python examples/run_isaac_sim_server.py  # Isaac Sim 환경 없이 실행 (실패)
cd /home/roarm_m3/dev_roarm/roarm_mcp && python examples/run_isaac_sim_server.py  # 잘못된 디렉토리
# Isaac Sim python.sh 없이 일반 Python으로 실행 (모듈 오류)
```

**🔄 환경 복구 프로토콜 (Isaac Sim 5.0 업데이트):**
```bash
# 1) Isaac Sim 5.0 환경 확인
echo "Isaac Sim 설치: $(ls /home/roarm_m3/isaac_sim 2>/dev/null && echo 'OK' || echo 'MISSING')"
ls /home/roarm_m3/isaac_sim/python.sh  # Isaac Sim Python 실행파일 확인

# 2) Isaac Sim PhysX Tensors 솔루션 확인
ls -la /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py

# 3) 프로젝트 구조 확인 (2025-10-02 구조)
cd /home/roarm_m3/dev_roarm/roarm_mcp
ls -la mcp/ isaac_sim_integration/ examples/ logs/

# 4) MCP 서버 프로세스 확인
ps aux | grep isaac_sim_server
ps aux | grep python | grep mcp

# 5) 포트 사용 확인
netstat -an | grep 8765
lsof -i :8765

# 6) 로그 확인
tail -f /home/roarm_m3/dev_roarm/roarm_mcp/logs/isaac_sim_mcp.log
```

### **2. 필수 문서 읽기 (작업 전 매번)**
- [ ] **README.md**: 프로젝트 개요 및 설치 가이드
- [ ] **daily_log/20251002.log**: 현재 작업 계획 및 진행상황
- [ ] **docs/** 디렉토리: 관련 기술 문서 및 구현 계획
- [ ] **config/*.yaml**: 시스템 설정 파일들

---

## 🔄 **PDCA 기반 체계적 작업 진행**

### **📋 Plan (계획) 단계**
```
1. 목표 명확화
   - Isaac Sim 통합 목표 정의
   - 로봇팔 제어 성공 기준 설정
   - MCP 프로토콜 구현 범위 설정
   - 강화학습 성능 목표 설정

2. 기술적 분석
   - Isaac Sim 의존성 확인
   - MCP 프로토콜 요구사항 분석
   - 로봇 모델 (UR10/Franka) 호환성 확인
   - 네트워크 통신 안정성 검토

3. 단계별 세분화
   - 시뮬레이션 환경 구축
   - MCP 서버/클라이언트 구현
   - 로봇 제어 인터페이스 개발
   - 강화학습 환경 통합
```

### **⚡ Do (실행) 단계**
```
1. 개발 순서 엄격 준수
   ✅ MCP 프로토콜 구현 (WebSocket 기반)
   ✅ MCP 서버/클라이언트 테스트 (통신 검증)
   ✅ Isaac Sim 통합 (시뮬레이션 환경)
   ✅ 로봇 제어 인터페이스 (UR10/Franka)
   ✅ 강화학습 환경 구축
   ✅ 통합 테스트 (E2E)

2. TDD 방식 적용 (신규 기능 시)
   ✅ 테스트 케이스 작성
   ✅ 실패 확인
   ✅ 구현
   ✅ 테스트 통과
   ✅ 리팩토링

3. 백업 파일 관리
   ✅ 중요 변경사항 전 백업 파일 생성
   ✅ 형식: [파일명]_backup_YYYYMMDD_HHMMSS.[확장자]
   ✅ backup/ 디렉토리 활용 (Git 제외)
   ✅ 작업 완료 후 백업 파일 정리
```

### **🔍 Check (검토) 단계**
```
1. 기능성 검증
   - MCP 프로토콜 메시지 교환 확인
   - Isaac Sim 시뮬레이션 정확성 검증
   - 로봇 제어 명령 응답성 확인
   - 강화학습 알고리즘 학습 진행 확인

2. 품질 기준 확인
   - 안전성: 로봇 동작 안전 범위 준수
   - 실시간성: 제어 지연 시간 최적화
   - 확장성: 다중 로봇 지원 가능성
   - 유지보수성: 모듈화된 코드 구조
   - 호환성: Isaac Sim 버전 호환성

3. 통합 테스트
   - MCP 서버-클라이언트 연동 테스트
   - Isaac Sim-로봇 제어 통합 확인
   - 강화학습-시뮬레이션 연동 검증
   - 전체 시스템 안정성 테스트
```

### **🚀 Action (개선) 단계**
```
1. 문제점 즉시 문서화
   - daily_log 파일 업데이트
   - 실수 사례 및 해결책 기록
   - 다음 작업을 위한 가이드 추가
   - GitHub Issues 생성 (필요시)

2. 프로세스 개선
   - 반복 오류 패턴 분석
   - Isaac Sim 설정 자동화 스크립트 개선
   - MCP 프로토콜 안정성 향상
   - 테스트 자동화 확대

3. 지식 공유
   - 기술적 발견사항 문서화
   - 로봇 제어 최적 설정 기록
   - Isaac Sim 활용 팁 정리
   - 강화학습 하이퍼파라미터 튜닝 결과 기록
```

---

## 🚨 **실수 방지 핵심 체크포인트**

### **🔧 개발 시 필수 확인사항**
- [ ] **Isaac Sim 환경**: Isaac Sim이 올바르게 설치되고 실행 가능한지 확인
- [ ] **포트 충돌**: MCP 서버 포트(기본 8765) 사용 가능 여부 확인
- [ ] **로봇 모델**: UR10/Franka 모델 파일 경로 및 로딩 상태 확인
- [ ] **네트워크 통신**: WebSocket 연결 안정성 및 메시지 전송 확인
- [ ] **GPU 메모리**: Isaac Sim 실행 시 GPU 메모리 사용량 모니터링

### **🛡️ 품질 보증 체크리스트**
- [ ] **안전성**: 로봇 동작 범위 제한 및 충돌 방지 구현
- [ ] **실시간성**: 제어 루프 주기 및 지연시간 최적화
- [ ] **확장성**: 다양한 로봇 모델 및 환경 지원
- [ ] **안정성**: 장시간 실행 시 메모리 누수 및 성능 저하 방지
- [ ] **호환성**: Isaac Sim 버전 및 의존성 라이브러리 호환성
- [ ] **재현성**: 강화학습 실험 결과 재현 가능성

### **💾 파일 관리 규칙**
- [ ] **백업 생성**: 중요 변경사항 전 backup/ 디렉토리에 백업
- [ ] **네이밍 규칙**: `[원본]_backup_YYYYMMDD_HHMMSS.[확장자]`
- [ ] **Git 제외**: backup/ 디렉토리는 .gitignore로 제외
- [ ] **버전 관리**: 완성된 코드만 main 브랜치에 커밋
- [ ] **daily_log 관리**: 매일 작업 내용 및 다음 계획 기록

---

## 📊 **핵심 시스템 정보 (암기 필수)**

### **🗂️ 시스템 경로 (2025-10-02 업데이트)**
```
프로젝트 루트: /home/roarm_m3/dev_roarm/roarm_mcp
Isaac Sim 5.0: /home/roarm_m3/isaac_sim
Isaac Sim 통합 솔루션: /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/
완성된 솔루션: isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py
테스트 아카이브: /home/roarm_m3/dev_roarm/roarm_mcp/tests_archive/
백업 디렉토리: /home/roarm_m3/dev_roarm/roarm_mcp/backup/
로그 디렉토리: /home/roarm_m3/dev_roarm/roarm_mcp/logs/
```

### **🔧 핵심 모듈 구조 (Isaac Sim 5.0 통합)**
```
MCP 프로토콜: mcp/ (protocol.py, server.py, client.py, isaac_sim_handler.py)
Isaac Sim 5.0 통합: isaac_sim_integration/ (완성된 PhysX Tensors 솔루션)
MCP 서버/클라이언트: examples/ (run_isaac_sim_server.py, isaac_sim_client.py)
로봇 제어: robot/ (controller.py, arms.py)
강화학습 환경: envs/ (robot_env.py)
설정 파일: config/ (mcp_config.yaml, robot_config.yaml, rl_config.yaml)
개발 히스토리: tests_archive/ (이전 테스트 파일들)
```

---

## **🚀 프로젝트 목표 (Isaac Sim 5.0 MCP 통합)**
RoArm 로봇을 Isaac Sim 5.0에서 제어하는 MCP (Model Context Protocol) 시스템 개발

### **📅 개발 단계 (현재 상태: 2025-10-02)**
1. **✅ Isaac Sim 5.0 연동 (완료)**
   - Isaac Sim 5.0 PhysX Tensors API 통합 완료
   - Isaac Sim MCP 핸들러 구현 완료 (mcp/isaac_sim_handler.py)
   - 클라우드 에셋 로딩 시스템 구축 완료
   - 완성된 솔루션: isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py

2. **🔄 MCP 서버 구축 (진행 중)**
   - 서버/클라이언트 구조 구현 완료
   - Isaac Sim 실행 환경 스크립트 생성 완료 (examples/)
   - 통합 테스트 및 검증 단계
   - **다음 작업**: Isaac Sim MCP 서버 통합 테스트

3. **📋 센서 데이터 통합 (다음 단계)**
   - 카메라, LiDAR 센서 데이터 처리
   - 상태 정보 실시간 전송
   - 환경 인식 기능 개발
   - Isaac Sim Sensors API 연동

4. **🎯 고급 제어 기능 (계획)**
   - 경로 추적 (trajectory tracking)
   - 역기구학 (inverse kinematics)
   - 충돌 회피 알고리즘
   - 실시간 피드백 제어

---

### **⚠️ 주의사항**
```
Isaac Sim: GPU 메모리 8GB+ 권장, CUDA 필수
MCP 서버: WebSocket 포트 8765 (기본값)
로봇 제어: 안전 범위 설정 필수
강화학습: 긴 학습 시간으로 인한 시스템 안정성 중요
네트워크: 실시간 제어를 위한 낮은 지연시간 필요
```

---

## 🎯 **작업 수행 템플릿**

### **매 작업 시작 시**
```markdown
## RoArm MCP 작업 시작 (YYYY-MM-DD HH:MM)

### 목표
- [ ] Isaac Sim 통합 목표
- [ ] MCP 프로토콜 구현 목표
- [ ] 로봇 제어 기능 목표
- [ ] 강화학습 환경 목표

### Plan
- [ ] Isaac Sim 환경 확인 완료
- [ ] MCP 프로토콜 요구사항 분석 완료
- [ ] 로봇 모델 호환성 확인 완료
- [ ] 단계별 구현 계획 수립
- [ ] 백업 파일 생성 (필요시)

### Do
- [ ] MCP 프로토콜 구현
- [ ] MCP 통신 테스트
- [ ] Isaac Sim 환경 구축
- [ ] 로봇 제어 인터페이스 개발
- [ ] 강화학습 환경 통합
- [ ] 시스템 통합 테스트

### Check
- [ ] 안전성 및 실시간성 확인
- [ ] 시뮬레이션 정확성 검증
- [ ] 통합 테스트 수행
- [ ] 성능 지표 측정

### Action
- [ ] daily_log 업데이트
- [ ] 문제점 및 해결책 문서화
- [ ] 프로세스 개선사항 적용
- [ ] GitHub 커밋 및 문서 정리
```

---

## 🔥 **긴급 상황 대응 (Isaac Sim 5.0 업데이트)**

### **Isaac Sim 5.0 관련 오류 발생 시**
1. **Isaac Sim 5.0 환경 진단**
   ```bash
   cd ~/isaac_sim
   ls -la python.sh  # Isaac Sim Python 실행파일 확인
   nvidia-smi  # GPU 상태 확인
   ./python.sh -c "import omni.isaac.core; print('Isaac Sim OK')"  # 모듈 테스트
   ```

2. **PhysX Tensors 솔루션 확인**
   ```bash
   cd /home/roarm_m3/isaac_sim
   ./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/isaac_sim_integration/solutions/isaac_sim_physx_tensors_solution.py
   # 위 명령이 실행되면 Isaac Sim 5.0 환경 정상
   ```

3. **Isaac Sim MCP 서버 재시작**
   ```bash
   # 기존 프로세스 종료
   pkill -f isaac_sim_server
   ps aux | grep isaac_sim  # 프로세스 확인
   
   # 새로 시작 (Isaac Sim 환경에서)
   cd /home/roarm_m3/isaac_sim
   ./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless
   ```

### **MCP 통신 오류 발생 시**
1. **포트 8765 확인**
   ```bash
   netstat -an | grep 8765  # 포트 사용 확인
   lsof -i :8765  # 포트 사용 프로세스 확인
   ```

2. **Isaac Sim MCP 서버-클라이언트 연결 테스트**
   ```bash
   # 터미널 1: Isaac Sim MCP 서버
   cd /home/roarm_m3/isaac_sim
   ./python.sh /home/roarm_m3/dev_roarm/roarm_mcp/examples/run_isaac_sim_server.py --robot-type ur10 --headless
   
   # 터미널 2: 클라이언트 테스트
   cd /home/roarm_m3/dev_roarm/roarm_mcp
   python examples/isaac_sim_client.py --server-url ws://localhost:8765 --robot-type ur10
   ```

3. **백업 복구** (필요시)
   ```bash
   cp backup/[백업파일] [원본파일]
   # 또는 tests_archive에서 이전 버전 복구
   cp tests_archive/[이전_버전] [현재_위치]
   ```

4. **로그 확인**
   ```bash
   tail -f /home/roarm_m3/dev_roarm/roarm_mcp/logs/isaac_sim_mcp.log
   ```

5. **문서 업데이트**
   - 오류 상황 및 해결책을 현재 프로젝트 구조에 맞게 기록

---

## ✅ **작업 완료 기준 (Isaac Sim 5.0 MCP 통합)**

### **완료 선언 전 필수 확인**
- [ ] Isaac Sim 5.0 MCP 서버-클라이언트 통신 테스트 통과
- [ ] Isaac Sim 5.0 PhysX Tensors 시뮬레이션 정상 작동 확인
- [ ] 로봇 제어 안전성 검증 완료 (UR10 기준)
- [ ] MCP 프로토콜 메시지 송수신 안정성 확인
- [ ] Isaac Sim 환경에서 서버 실행 안정성 확인
- [ ] 통합 테스트 수행 완료 (examples/ 스크립트 기준)
- [ ] 프로젝트 문서 업데이트 완료 (현재 구조 반영)
- [ ] 백업 및 tests_archive 정리 완료 (필요시)
- [ ] GitHub 커밋 및 문서 업데이트 완료
- [ ] 다음 작업 계획 수립 완료 (센서 데이터 통합 단계)

### **현재 단계별 검증 기준 (2025-10-02)**
**단계 1 - Isaac Sim 5.0 연동: ✅ 완료**
- [x] PhysX Tensors API 통합 완료
- [x] Isaac Sim MCP 핸들러 구현 완료
- [x] 완성된 솔루션 생성 완료

**단계 2 - MCP 서버 구축: 🔄 진행 중**
- [x] 서버/클라이언트 구조 구현 완료
- [x] Isaac Sim 실행 환경 스크립트 생성 완료
- [ ] **다음**: Isaac Sim MCP 서버 통합 테스트

**단계 3 - 센서 데이터 통합: 📋 대기 중**
**단계 4 - 고급 제어 기능: 🎯 계획 단계**

---

> **⚠️ 이 문서는 RoArm MCP 프로젝트 전용 작업 수행 원칙입니다**  
> **모든 작업 시작 전 반드시 읽고 준수하세요**  
> **Isaac Sim과 로봇 제어의 안전성과 품질 향상을 위한 필수 가이드입니다**