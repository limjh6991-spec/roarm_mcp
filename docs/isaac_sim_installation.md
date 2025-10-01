# Isaac Sim 설치 가이드 (NVIDIA Omniverse)
## 시스템 요구사항

- **운영체제**: Ubuntu 22.04 LTS 권장
- **GPU**: NVIDIA GeForce RTX 시리즈 (RTX 2000 시리즈 이상)
- **드라이버**: NVIDIA 드라이버 525 이상
- **RAM**: 최소 16GB (32GB 이상 권장)
- **저장공간**: 최소 30GB (SSD 권장)
- **CPU**: 8 코어 이상 권장

## 설치 단계

1. NVIDIA 계정 생성: https://developer.nvidia.com/

2. NVIDIA Omniverse Launcher 다운로드: https://www.nvidia.com/ko-kr/omniverse/

3. Omniverse Launcher 설치 및 실행

4. Launcher에서 'Exchange' 탭으로 이동하여 'Isaac Sim' 애플리케이션 찾기

5. Isaac Sim 다운로드 및 설치

6. Isaac Sim 실행 및 라이센스 활성화

## MCP 확장 설치

```bash
# uv/uvx 설치
curl -LsSf https://astral.sh/uv/install.sh | sh

# mcp 설치
uv pip install "mcp[cli]"

# isaac-sim-mcp 저장소 클론
cd ~/Documents
git clone https://github.com/omni-mcp/isaac-sim-mcp
```

## Isaac Sim 확장 활성화

```bash
# Isaac Sim 디렉토리로 이동
cd ~/.local/share/ov/pkg/isaac-sim-<버전>

# 확장 활성화
./isaac-sim.sh --ext-folder ~/Documents/isaac-sim-mcp/ --enable isaac.sim.mcp_extension
```

## 환경 변수 설정 (선택사항)

필요에 따라 다음 환경 변수를 설정합니다:

```bash
# Beaver3D 모델 이름 설정 (선택사항)
export BEAVER3D_MODEL=<beaver3d 모델명>

# Beaver3D API 키 설정 (선택사항)
export ARK_API_KEY=<Beaver3D API 키>

# NVIDIA API 키 설정 (선택사항)
export NVIDIA_API_KEY="<NVIDIA API 키>"
```

## MCP 서버 실행

```bash
# MCP 서버 실행
uv run <isaac-sim-mcp 경로>/isaac_mcp/server.py
```

## 확장 서버 연결 확인

Isaac Sim 실행 후 확장이 활성화되면 로그에서 다음과 같은 메시지를 확인할 수 있습니다:

```
[7.160s] [ext: isaac.sim.mcp_extension-0.1.0] startup
trigger on_startup for: isaac.sim.mcp_extension-0.1.0
Isaac Sim MCP server started on localhost:8766
```

기본적으로 확장 서버는 **localhost:8766**에서 수신 대기합니다.
