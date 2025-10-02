#!/usr/bin/env python3

"""
Isaac Sim 5.0 기본 환경 테스트 스크립트 (업데이트된 API)
RoArm MCP 프로젝트용
"""

import sys
import os

def test_isaac_sim_imports_v5():
    """Isaac Sim 5.0 모듈 임포트 테스트"""
    print("=== Isaac Sim 5.0 모듈 임포트 테스트 ===")
    
    try:
        # Isaac Sim 5.0의 새로운 API
        from isaacsim import SimulationApp
        print("✅ isaacsim.SimulationApp 임포트 성공")
        
        return True
        
    except ImportError as e:
        print(f"❌ 임포트 오류: {e}")
        return False
    except Exception as e:
        print(f"❌ 기타 오류: {e}")
        return False

def test_isaac_sim_initialization_v5():
    """Isaac Sim 5.0 초기화 테스트"""
    print("\n=== Isaac Sim 5.0 초기화 테스트 ===")
    
    try:
        # 헤드리스 모드로 SimulationApp 시작
        from isaacsim import SimulationApp
        
        # 헤드리스 모드 설정
        simulation_app = SimulationApp({
            "headless": True,  # GUI 없이 실행
            "width": 1024,
            "height": 768,
        })
        
        print("✅ SimulationApp 초기화 성공")
        
        # 기본 Isaac Lab 모듈들 테스트
        try:
            import omni.isaac.lab.sim as sim_utils
            print("✅ Isaac Lab sim 모듈 임포트 성공")
        except ImportError:
            print("⚠️ Isaac Lab sim 모듈을 찾을 수 없습니다")
        
        try:
            import omni.isaac.lab.utils.configclass as configclass
            print("✅ Isaac Lab configclass 모듈 임포트 성공")
        except ImportError:
            print("⚠️ Isaac Lab configclass 모듈을 찾을 수 없습니다")
        
        # USD 및 Physics 모듈 테스트
        try:
            import omni.usd
            print("✅ omni.usd 모듈 임포트 성공")
            
            stage = omni.usd.get_context().get_stage()
            print("✅ USD Stage 생성 성공")
        except ImportError as e:
            print(f"⚠️ USD 모듈 오류: {e}")
        
        # 시뮬레이션 앱 정리
        simulation_app.close()
        print("✅ Isaac Sim 정리 완료")
        
        return True
        
    except Exception as e:
        print(f"❌ Isaac Sim 초기화 오류: {e}")
        import traceback
        traceback.print_exc()
        try:
            simulation_app.close()
        except:
            pass
        return False

def main():
    """메인 테스트 함수"""
    print("🤖 RoArm MCP Isaac Sim 5.0 기본 환경 테스트 시작")
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
    import_success = test_isaac_sim_imports_v5()
    
    if import_success:
        init_success = test_isaac_sim_initialization_v5()
        
        if init_success:
            print("\n🎉 모든 Isaac Sim 5.0 기본 테스트가 성공했습니다!")
            return True
        else:
            print("\n❌ Isaac Sim 5.0 초기화 테스트에 실패했습니다")
            return False
    else:
        print("\n❌ Isaac Sim 5.0 모듈 임포트에 실패했습니다")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)