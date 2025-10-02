#!/usr/bin/env python3
"""
RoArm MCP - Isaac Sim 5.0 연동 통합 테스트 (타입 미스매치 수정된 버전)
"""

import sys
import time
print("🤖 RoArm MCP - Isaac Sim 5.0 연동 통합 테스트 (타입 미스매치 수정된 버전)")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {sys.path[0] if sys.path else 'N/A'}")

def test_mcp_isaac_integration_fixed():
    """Isaac Sim 5.0과 RoArm MCP 연동 테스트 (타입 미스매치 수정된 버전)"""
    
    # Isaac Sim 5.0 앱 초기화
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})
    
    try:
        print("=== Isaac Sim 5.0과 RoArm MCP 연동 테스트 (타입 미스매치 수정된 버전) ===")
        print("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # USD 관련 모듈들 import (SimulationApp 초기화 후에 해야 함)
        from pxr import Usd, UsdGeom, UsdLux, Sdf
        import omni.usd
        
        # USD Context 가져오기
        ctx = omni.usd.get_context()
        
        # Stage 생성 (올바른 방법)
        created = ctx.new_stage()  # created는 bool 값 (성공 여부)
        if not created:
            raise RuntimeError("USD Stage 생성 실패")
        
        # 실제 Stage 객체는 get_stage()로 가져와야 함
        stage = ctx.get_stage()
        if stage is None:
            raise RuntimeError("ctx.get_stage()가 None을 반환했습니다")
        
        # 타입 검증 (방어 코드)
        assert isinstance(stage, Usd.Stage), f"stage는 Usd.Stage 타입이어야 하는데 {type(stage)}입니다"
        print("✅ USD Stage 생성 성공 (타입 검증 완료)")
        
        # World Prim 생성 (권장 패턴)
        if stage.GetDefaultPrim() is None:
            world_prim = stage.DefinePrim("/World", "Xform")
            stage.SetDefaultPrim(world_prim)
            print("✅ World Prim 생성 및 기본 Prim 설정 완료")
        
        # 조명 추가 (올바른 방법)
        light_path = Sdf.Path("/World/DistantLight")  # 명시적으로 Sdf.Path 사용
        light_prim = UsdLux.DistantLight.Define(stage, light_path)
        
        # 조명 속성 설정
        distant_light = UsdLux.DistantLight(stage.GetPrimAtPath(light_path))
        distant_light.CreateIntensityAttr(5000.0)
        distant_light.CreateAngleAttr(0.53)  # 태양광 느낌
        print("✅ USD DistantLight 추가 성공 (올바른 타입으로)")
        
        # 기본 환경 설정
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        print("✅ USD Stage 축 및 단위 설정 완료")
        
        # 물리 월드 생성 테스트
        from omni.physx import acquire_physx_interface
        physx = acquire_physx_interface()
        
        # World 인터페이스 테스트 (Isaac Sim 통합 환경)
        from omni.isaac.core import World
        world = World.instance()
        if world is None:
            world = World()  # stage 파라미터 제거
        print("✅ Isaac Sim World 인터페이스 초기화 성공")
        
        # 물리 시뮬레이션 설정 (World를 통해)
        world.get_physics_context().enable_gpu_dynamics(flag=True)
        world.get_physics_context().set_physics_dt(1.0/60.0)
        print("✅ PhysX 물리 엔진 설정 완료")
        
        # 시뮬레이션 단계별 실행 테스트 (올바른 방법)
        simulation_steps = 10
        for step in range(simulation_steps):
            world.step(render=False)  # World.step() 사용
            if step % 5 == 0:
                print(f"  📊 시뮬레이션 스텝: {step + 1}/{simulation_steps}")
        
        print("✅ 물리 시뮬레이션 단계별 실행 성공")
        
        # 기본 Scene 구성 요소 추가
        from pxr import UsdPhysics
        
        # 바닥 생성
        ground_path = "/World/GroundPlane"
        ground_geom = UsdGeom.Mesh.Define(stage, ground_path)
        ground_geom.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
        ground_geom.CreateFaceVertexCountsAttr([4])
        ground_geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        
        # 물리 속성 추가
        ground_prim = stage.GetPrimAtPath(ground_path)
        UsdPhysics.CollisionAPI.Apply(ground_prim)
        print("✅ 바닥 Mesh 및 충돌 설정 완료")
        
        # USD 파일 저장 (선택사항)
        output_path = "/home/roarm_m3/dev_roarm/roarm_mcp/tests/isaac_test_scene.usd"
        try:
            stage.GetRootLayer().Export(output_path)
            print(f"✅ USD 씬 파일 저장 완료: {output_path}")
        except Exception as e:
            print(f"⚠️ USD 파일 저장 실패: {str(e)}")
        
        # MCP 연동을 위한 기본 설정 완료 확인
        print("🔗 Isaac Sim 5.0과 MCP 연동 기본 환경 구성 완료")
        print("📝 다음 단계: 로봇 모델 로딩 및 제어 테스트")
        
        return True
        
    except Exception as e:
        print(f"❌ Isaac Sim-MCP 연동 오류: {str(e)}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 정리 작업 (try/finally로 보장)
        try:
            simulation_app.close()
            print("🔄 Isaac Sim 종료 완료")
        except Exception as cleanup_error:
            print(f"⚠️ 정리 작업 중 오류: {cleanup_error}")

if __name__ == "__main__":
    success = test_mcp_isaac_integration_fixed()
    exit(0 if success else 1)