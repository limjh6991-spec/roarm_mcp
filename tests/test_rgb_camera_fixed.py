#!/usr/bin/env python3
"""
Isaac Sim 5.0 RGB 카메라 테스트 (UsdLux 스키마 수정)
올바른 USD 스키마와 마운트 구조를 사용한 RGB 카메라 구현
"""

import sys
import os
import numpy as np
from pathlib import Path

# Python 및 Isaac Sim 환경 정보 출력
print("🎥 Isaac Sim 5.0 RGB 카메라 테스트 (UsdLux 수정)")
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
        
        # USD 및 Omni 모듈 import (UsdLux 추가)
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        import omni.kit.commands
        
        print("✅ USD 모듈 import 성공 (UsdLux 포함)")
        
        # USD 스테이지 가져오기
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            # 새 스테이지 생성
            stage = Usd.Stage.CreateNew("memory_stage.usd")
            usd_context.attach_stage(stage)
            print("  - 새 USD 스테이지 생성")
        
        # 스테이지 단위를 미터로 설정
        stage.SetMetadata("metersPerUnit", 1.0)
        
        # World 프림 생성 (루트)
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        print("🌍 기본 환경 설정...")
        
        # 조명 마운트 생성 (권장 방식)
        lights_mount = stage.DefinePrim(Sdf.Path("/World/Lights"), "Xform")
        lights_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(lights_mount))
        lights_xform.SetTranslate(Gf.Vec3d(0, 0, 3))  # 포즈는 마운트에만
        
        # UsdLux로 조명 생성
        distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/DistantLight"))
        distant_light.CreateIntensityAttr(5000.0)  # 밝기 설정
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))  # 흰색 조명
        
        print("  - UsdLux 거리 조명 설정 완료")
        
        # 환경광 추가 (DomeLight)
        dome_light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/DomeLight"))
        dome_light.CreateIntensityAttr(1000.0)
        dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.9, 1.0))  # 약간 파란빛 환경광
        
        print("  - UsdLux 환경광(DomeLight) 설정 완료")
        
        # 바닥 평면 생성
        ground_geom = UsdGeom.Cube.Define(stage, Sdf.Path("/World/GroundPlane"))
        ground_geom.CreateSizeAttr(1.0)
        ground_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))
        ground_geom.AddScaleOp().Set(Gf.Vec3f(10, 10, 1))
        
        print("  - 바닥 평면 추가 완료")
        
        # 테스트용 큐브 객체 추가
        test_cube = UsdGeom.Cube.Define(stage, Sdf.Path("/World/TestCube"))
        test_cube.CreateSizeAttr(1.0)
        test_cube.AddTranslateOp().Set(Gf.Vec3f(2, 0, 1))
        test_cube.AddScaleOp().Set(Gf.Vec3f(1, 1, 1))
        
        print("  - 테스트 큐브 추가 완료")
        
        # 센서 마운트 및 카메라 생성 (권장 방식)
        print("📷 RGB 카메라 설정...")
        
        # 센서 그룹 생성
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        
        # 카메라 마운트 생성
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(-5.0, 0.0, 2.0))  # 카메라 위치
        cam_mount_xform.SetRotate(Gf.Vec3f(0, 15, 0))  # 카메라 각도
        
        # RGB 카메라 프림 생성 (마운트 하위에)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        
        # 카메라 속성 설정
        camera_prim.CreateHorizontalApertureAttr(36.0)  # mm
        camera_prim.CreateVerticalApertureAttr(24.0)    # mm
        camera_prim.CreateFocalLengthAttr(24.0)         # mm
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))  # Near/Far 클리핑
        
        print("  - USD 카메라 프림 및 마운트 생성 완료")
        
        # Replicator를 사용한 렌더 제품 생성
        try:
            import omni.replicator.core as rep
            
            # 렌더 제품 생성 (카메라에서 이미지 캡처)
            render_product = rep.create.render_product("/World/Sensors/FrontCam_Mount/FrontCam", (1280, 720))
            
            # 출력 디렉토리 설정
            output_dir = "/home/roarm_m3/tmp/rgb_test"
            os.makedirs(output_dir, exist_ok=True)
            
            # BasicWriter로 RGB 이미지 저장 설정
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(
                output_dir=output_dir,
                rgb=True,
                camera_params=True
            )
            writer.attach([render_product])
            
            print(f"  - Replicator 렌더 제품 생성 완료 (출력: {output_dir})")
            
        except ImportError as e:
            print(f"  ⚠️ Replicator 사용 불가: {e}")
        
        print("✅ RGB 카메라 설정 완료")
        
        # 시뮬레이션 업데이트 및 렌더링
        print("🔄 시뮬레이션 업데이트 및 렌더링...")
        
        # 업데이트 명령 실행
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        for i in range(20):  # 더 많은 프레임 업데이트
            app.update()
            if i % 5 == 0:
                print(f"  - 프레임 {i+1}/20 업데이트")
        
        # 추가적인 렌더링 시도
        try:
            # Replicator 렌더링 트리거
            if 'rep' in locals():
                rep.orchestrator.step()
                print("  - Replicator 렌더링 단계 실행")
        except Exception as e:
            print(f"  ⚠️ Replicator 렌더링 오류: {e}")
        
        print("✅ RGB 카메라 테스트 완료")
        print("\n📊 테스트 결과:")
        print("  - Isaac Sim 5.0 초기화: 성공 ✅")
        print("  - USD 스테이지 생성: 성공 ✅")
        print("  - UsdLux 조명 시스템: 성공 ✅")
        print("  - RGB 카메라 마운트 구조: 성공 ✅")
        print("  - 카메라 프림 생성: 성공 ✅")
        print("  - 환경 설정: 성공 ✅")
        print("  - 시뮬레이션 업데이트: 성공 ✅")
        
        # 생성된 파일 확인
        if os.path.exists("/home/roarm_m3/tmp/rgb_test"):
            files = list(Path("/home/roarm_m3/tmp/rgb_test").glob("**/*"))
            if files:
                print(f"  - 생성된 파일: {len(files)}개")
                for f in files[:5]:  # 처음 5개 파일만 표시
                    print(f"    * {f}")
            else:
                print("  - 아직 이미지 파일이 생성되지 않음")
        
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