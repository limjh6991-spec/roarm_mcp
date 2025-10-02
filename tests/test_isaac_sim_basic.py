#!/usr/bin/env python3

"""
Isaac Sim 기본 환경 테스트 스크립트
RoArm MCP 프로젝트용
"""

import sys
import os

# Isaac Sim Python 환경 설정
sys.path.append('/home/roarm_m3/isaac_sim')

def test_isaac_sim_imports():
    """Isaac Sim 모듈 임포트 테스트"""
    print("=== Isaac Sim 모듈 임포트 테스트 ===")
    
    try:
        # 기본 Isaac Sim 모듈들
        import omni
        print("✅ omni 모듈 임포트 성공")
        
        from omni.isaac.kit import SimulationApp
        print("✅ SimulationApp 임포트 성공")
        
        # Core 모듈들
        from omni.isaac.core import World
        print("✅ World 임포트 성공")
        
        from omni.isaac.core.objects import DynamicCuboid
        print("✅ DynamicCuboid 임포트 성공")
        
        return True
        
    except ImportError as e:
        print(f"❌ 임포트 오류: {e}")
        return False
    except Exception as e:
        print(f"❌ 기타 오류: {e}")
        return False

def test_isaac_sim_initialization():
    """Isaac Sim 초기화 테스트"""
    print("\n=== Isaac Sim 초기화 테스트 ===")
    
    try:
        # 헤드리스 모드로 SimulationApp 시작
        from omni.isaac.kit import SimulationApp
        
        # 헤드리스 모드 설정
        simulation_app = SimulationApp({
            "headless": True,  # GUI 없이 실행
            "anti_aliasing": 0,
            "multi_gpu": False,
        })
        
        print("✅ SimulationApp 초기화 성공")
        
        # World 생성
        from omni.isaac.core import World
        world = World()
        print("✅ World 생성 성공")
        
        # 간단한 큐브 생성 테스트
        from omni.isaac.core.objects import DynamicCuboid
        cube = DynamicCuboid(
            prim_path="/World/Cube",
            name="test_cube",
            size=1.0,
            color=[1.0, 0.0, 0.0]  # 빨간색
        )
        print("✅ 큐브 생성 성공")
        
        # 시뮬레이션 리셋
        world.reset()
        print("✅ 시뮬레이션 리셋 성공")
        
        # 몇 스텝 실행
        for i in range(5):
            world.step(render=False)
        print("✅ 시뮬레이션 스텝 실행 성공")
        
        # 정리
        simulation_app.close()
        print("✅ Isaac Sim 정리 완료")
        
        return True
        
    except Exception as e:
        print(f"❌ Isaac Sim 초기화 오류: {e}")
        try:
            simulation_app.close()
        except:
            pass
        return False

def main():
    """메인 테스트 함수"""
    print("🤖 RoArm MCP Isaac Sim 기본 환경 테스트 시작")
    print(f"Python 버전: {sys.version}")
    print(f"Isaac Sim 경로: /home/roarm_m3/isaac_sim")
    
    # GPU 상태 확인
    try:
        import subprocess
        gpu_info = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader'], 
                                 capture_output=True, text=True)
        if gpu_info.returncode == 0:
            print(f"GPU 정보: {gpu_info.stdout.strip()}")
        else:
            print("⚠️ GPU 정보를 확인할 수 없습니다")
    except:
        print("⚠️ nvidia-smi를 실행할 수 없습니다")
    
    print()
    
    # 테스트 실행
    import_success = test_isaac_sim_imports()
    
    if import_success:
        init_success = test_isaac_sim_initialization()
        
        if init_success:
            print("\n🎉 모든 Isaac Sim 기본 테스트가 성공했습니다!")
            return True
        else:
            print("\n❌ Isaac Sim 초기화 테스트에 실패했습니다")
            return False
    else:
        print("\n❌ Isaac Sim 모듈 임포트에 실패했습니다")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)