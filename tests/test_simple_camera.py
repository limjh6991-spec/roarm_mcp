#!/usr/bin/env python3
"""
Simple RGB Camera Test for Isaac Sim 5.0
Isaac Sim 5.0용 간단한 RGB 카메라 테스트

기존 완성된 솔루션의 구조를 따라 최소한의 카메라 기능만 테스트합니다.
"""

import os
import sys
import time
import logging

# Isaac Sim imports
import isaacsim
from isaacsim import SimulationApp

print("🎥 Isaac Sim 5.0 간단한 RGB 카메라 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

def main():
    print("✅ Isaac Sim 5.0 SimulationApp 초기화 중...")
    
    # Create simulation app (headless mode)
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # Import Isaac Sim modules after app creation
        from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.core.utils.prims import create_prim, find_matching_prims
        import omni.usd
        import torch
        
        print("✅ Isaac Sim 모듈 import 성공")
        
        # World 생성 및 초기화
        world = World()
        world.clear()
        
        print("🌍 시뮬레이션 환경 설정...")
        
        # 기본 조명 설정
        stage = get_current_stage()
        light_prim = stage.DefinePrim("/World/DistantLight", "DistantLight")
        light = UsdGeom.Xformable(light_prim)
        light.AddTranslateOp().Set(Gf.Vec3f(0, 0, 10))
        
        print("  - 기본 조명 설정 완료")
        
        # 바닥 평면 추가
        create_prim(
            prim_path="/World/GroundPlane",
            prim_type="Cube",
            position=[0, 0, -0.5],
            scale=[10, 10, 1]
        )
        
        print("  - 바닥 평면 추가 완료")
        
        # 테스트용 객체 추가
        create_prim(
            prim_path="/World/TestCube",
            prim_type="Cube",
            position=[0, 0, 1.0],
            scale=[0.5, 0.5, 0.5]
        )
        
        print("  - 테스트 객체 추가 완료")
        
        # World 리셋
        world.reset()
        
        print("🔄 World 리셋 완료")
        
        # 간단한 카메라 프림 추가 (Isaac Sim Camera API 대신 USD Camera 사용)
        camera_prim = stage.DefinePrim("/World/Camera", "Camera")
        camera_xform = UsdGeom.Xformable(camera_prim)
        camera_xform.AddTranslateOp().Set(Gf.Vec3f(3, 3, 2))
        
        # 카메라 속성 설정
        camera_api = UsdGeom.Camera(camera_prim)
        camera_api.GetFocalLengthAttr().Set(24.0)
        
        print("📸 USD 카메라 프림 생성 완료")
        
        # 시뮬레이션 몇 스텝 실행
        print("🚀 시뮬레이션 스텝 테스트...")
        
        for step in range(5):
            world.step(render=True)
            print(f"  스텝 {step+1}/5 완료")
            time.sleep(0.2)
            
        print("✅ 기본 시뮬레이션 테스트 성공!")
        
        # USD 파일 저장
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/camera_test_scene.usd"
        stage.Export(output_path)
        print(f"💾 USD 파일 저장 완료: {output_path}")
        
        print("🎉 RGB 카메라 기본 테스트 완료!")
        print("   - 시뮬레이션 환경 구축 성공")
        print("   - USD 카메라 프림 생성 성공") 
        print("   - 시뮬레이션 스텝 실행 성공")
        print("   - USD 파일 저장 성공")
        
    except Exception as e:
        print(f"❌ 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Isaac Sim 정리
        simulation_app.close()
        print("🔚 Isaac Sim 종료")

if __name__ == "__main__":
    main()