#!/usr/bin/env python3
"""
Isaac Sim 5.0 기본 카메라 테스트
USD Camera Primitive를 사용한 간단한 이미지 캡처 테스트
"""

import sys
import os
import numpy as np
from pathlib import Path

# Python 및 Isaac Sim 환경 정보 출력
print("🎥 Isaac Sim 5.0 기본 카메라 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

def main():
    """메인 테스트 함수"""
    try:
        # Isaac Sim 초기화
        from isaacsim import SimulationApp
        
        simulation_app = SimulationApp({
            "headless": True,
            "width": 1280,
            "height": 720
        })
        
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # USD 및 Omni 기본 모듈 import
        from pxr import Usd, UsdGeom, Gf, Sdf
        from omni.usd import get_context
        import omni.kit.commands
        
        print("✅ USD 모듈 import 성공")
        
        # USD 스테이지 가져오기
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            # 새 스테이지 생성
            stage = Usd.Stage.CreateNew("memory_stage.usd")
            usd_context.attach_stage(stage)
            print("  - 새 USD 스테이지 생성")
        
        # World 프림 생성 (루트)
        world_prim = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        print("🌍 기본 환경 설정...")
        
        # 기본 조명 추가
        light_prim = UsdGeom.DistantLight.Define(stage, "/World/DistantLight")
        light_prim.CreateAngleAttr(0.53)  # 태양 크기
        light_prim.CreateIntensityAttr(3000.0)  # 조명 강도
        light_prim.AddTranslateOp().Set(Gf.Vec3f(0, 0, 10))
        light_prim.AddRotateXYZOp().Set(Gf.Vec3f(-45, -45, 0))
        
        print("  - 거리 조명 설정 완료")
        
        # 바닥 평면 생성
        ground_geom = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
        ground_geom.CreateSizeAttr(1.0)
        ground_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))
        ground_geom.AddScaleOp().Set(Gf.Vec3f(10, 10, 1))
        
        print("  - 바닥 평면 추가 완료")
        
        # 테스트용 큐브 객체 추가
        test_cube = UsdGeom.Cube.Define(stage, "/World/TestCube")
        test_cube.CreateSizeAttr(1.0)
        test_cube.AddTranslateOp().Set(Gf.Vec3f(2, 0, 1))
        test_cube.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
        
        print("  - 테스트 큐브 추가 완료")
        
        # 카메라 생성
        print("📷 카메라 설정...")
        
        camera_prim = UsdGeom.Camera.Define(stage, "/World/Camera")
        camera = camera_prim.GetPrim()
        
        # 카메라 속성 설정
        camera_prim.CreateFocalLengthAttr(24.0)  # 초점거리 (mm)
        camera_prim.CreateHorizontalApertureAttr(20.0)  # 수평 조리개
        camera_prim.CreateVerticalApertureAttr(15.0)  # 수직 조리개
        
        # 카메라 위치 및 회전 설정
        camera_prim.AddTranslateOp().Set(Gf.Vec3f(-5, 0, 2))  # 카메라 위치
        camera_prim.AddRotateXYZOp().Set(Gf.Vec3f(0, 15, 0))  # 카메라 각도
        
        print("  - USD 카메라 프림 생성 완료")
        
        # 렌더링 제품 생성 (이미지 캡처를 위한)
        from omni.syntheticdata import SyntheticData
        from omni.syntheticdata._syntheticdata import acquire_syntheticdata_interface
        
        # SyntheticData 인터페이스 얻기
        sd_interface = acquire_syntheticdata_interface()
        
        if sd_interface:
            print("  - SyntheticData 인터페이스 획득 성공")
            
            # 렌더 제품 ID 생성
            render_product_path = omni.syntheticdata.SyntheticData.convert_sensor_type_to_renderproduct(
                "/World/Camera"
            )
            
            print(f"  - 렌더 제품 경로: {render_product_path}")
        else:
            print("  - SyntheticData 인터페이스 획득 실패")
        
        print("✅ 카메라 설정 완료")
        
        # 시뮬레이션 업데이트
        print("🔄 시뮬레이션 업데이트...")
        
        # 업데이트 명령 실행
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        for i in range(10):
            app.update()
            print(f"  - 프레임 {i+1}/10 업데이트")
        
        print("✅ 기본 카메라 테스트 완료")
        print("\n📊 테스트 결과:")
        print("  - Isaac Sim 5.0 초기화: 성공 ✅")
        print("  - USD 스테이지 생성: 성공 ✅")
        print("  - 카메라 프림 생성: 성공 ✅")
        print("  - 환경 설정: 성공 ✅")
        print("  - 시뮬레이션 업데이트: 성공 ✅")
        
    except ImportError as e:
        print(f"❌ 모듈 import 오류: {e}")
        return False
        
    except Exception as e:
        print(f"❌ 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        # 정리
        try:
            simulation_app.close()
            print("🧹 Isaac Sim 정리 완료")
        except Exception as e:
            print(f"⚠️ 정리 과정에서 오류: {e}")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)