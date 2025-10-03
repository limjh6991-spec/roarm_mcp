#!/usr/bin/env python3
"""
RGB Camera Sensor Test for Isaac Sim 5.0
Isaac Sim 5.0 환경에서 RGB 카메라 센서 테스트

이 스크립트는 Isaac Sim 5.0 환경에서 RGB 카메라 센서를
테스트하고 기본 이미지 캡처 기능을 검증합니다.
"""

import sys
import os
import numpy as np
import logging
import time
import json

# 프로젝트 경로 추가
project_root = '/home/roarm_m3/dev_roarm/roarm_mcp'
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Isaac Sim imports - 완성된 솔루션 방식 적용
import isaacsim
from isaacsim import SimulationApp

print("🎥 Isaac Sim 5.0 RGB 카메라 센서 테스트")
print(f"Python 버전: {sys.version}")
print(f"작업 디렉토리: {os.getcwd()}")

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

import sys
import os
import numpy as np
import logging
import time
import json

# 프로젝트 경로 추가
project_root = '/home/roarm_m3/dev_roarm/roarm_mcp'
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Isaac Sim imports - 기존 완성 솔루션 참고
import omni.isaac.core
from omni.isaac.core import World

# Isaac Sim 시뮬레이션 앱 없이 직접 실행 (기존 솔루션 방식)

# Isaac Sim 초기화 후 다른 모듈들 import
import omni.isaac.core
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim
import omni.usd
from pxr import UsdGeom, Gf
import torch

# 프로젝트 센서 모듈
from sensors.camera_sensor import RGBCameraSensor, create_rgb_camera

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def setup_test_scene():
    """테스트 씬 설정"""
    logger.info("🌍 Isaac Sim 테스트 씬 설정 중...")
    
    # Isaac Sim imports after app creation
    from pxr import Usd, UsdPhysics, UsdGeom, Gf, Sdf
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.prims import create_prim
    import omni.usd
    import torch
    
    # World 생성
    world = World()
    world.clear()
    
    # 기본 조명 추가
    create_prim(
        prim_path="/World/DistantLight",
        prim_type="DistantLight",
        position=np.array([0, 0, 10]),
        attributes={"intensity": 3000.0}
    )
    
    # 바닥 평면 추가
    create_prim(
        prim_path="/World/GroundPlane",
        prim_type="Cube",
        position=np.array([0, 0, -0.5]),
        scale=np.array([10, 10, 1])
    )
    
    # 테스트용 큐브 객체들 추가
    for i in range(3):
        x = i * 2.0 - 2.0  # -2, 0, 2
        create_prim(
            prim_path=f"/World/TestCube_{i}",
            prim_type="Cube",
            position=np.array([x, 0, 1.0]),
            scale=np.array([0.5, 0.5, 0.5])
        )
    
    # World 초기화
    world.reset()
    logger.info("✅ 테스트 씬 설정 완료")
    
    return world

def test_rgb_camera_sensor():
    """RGB 카메라 센서 테스트"""
    logger.info("🎥 RGB 카메라 센서 테스트 시작")
    
    try:
        # RGB 카메라 생성
        camera = create_rgb_camera(
            prim_path="/World/RGBCamera",
            position=(4.0, 4.0, 3.0),  # 카메라 위치
            target=(0.0, 0.0, 1.0),    # 바라볼 지점
            resolution=(640, 480),      # 테스트용 해상도
            frequency=10.0             # 10 FPS
        )
        
        # 카메라 초기화
        if not camera.initialize():
            logger.error("❌ 카메라 초기화 실패")
            return False
            
        # 카메라 정보 출력
        camera_info = camera.get_camera_info()
        logger.info(f"📋 카메라 정보:\n{json.dumps(camera_info, indent=2)}")
        
        # 시뮬레이션 몇 스텝 실행하여 렌더링 안정화
        world = World.instance()
        for i in range(5):
            world.step(render=True)
            time.sleep(0.1)
            
        logger.info("🎬 이미지 캡처 테스트 시작...")
        
        # 여러 이미지 캡처 테스트
        success_count = 0
        total_attempts = 5
        
        for i in range(total_attempts):
            logger.info(f"📸 이미지 캡처 시도 {i+1}/{total_attempts}")
            
            # RGB 이미지 캡처
            rgb_image = camera.capture_rgb_image()
            if rgb_image is not None:
                success_count += 1
                logger.info(f"✅ 캡처 성공: 이미지 크기 {rgb_image.shape}")
                
                # 이미지 통계 정보
                mean_rgb = np.mean(rgb_image, axis=(0, 1))
                logger.info(f"   RGB 평균값: R={mean_rgb[0]:.1f}, G={mean_rgb[1]:.1f}, B={mean_rgb[2]:.1f}")
                
                # Base64 인코딩 테스트
                encoded_image = camera.capture_and_encode_image(quality=85)
                if encoded_image:
                    logger.info(f"🔐 Base64 인코딩 성공: {len(encoded_image)} 문자")
                else:
                    logger.warning("⚠️ Base64 인코딩 실패")
            else:
                logger.warning(f"⚠️ 이미지 캡처 실패 ({i+1}번째 시도)")
                
            # 시뮬레이션 스텝
            world.step(render=True)
            time.sleep(0.2)
            
        # 성능 메트릭 확인
        performance = camera.get_performance_metrics()
        logger.info(f"📊 성능 메트릭: {json.dumps(performance, indent=2)}")
        
        # 정리
        camera.cleanup()
        
        # 결과 요약
        success_rate = success_count / total_attempts * 100
        logger.info(f"🎯 테스트 결과: {success_count}/{total_attempts} 성공 ({success_rate:.1f}%)")
        
        if success_count > 0:
            logger.info("🎉 RGB 카메라 센서 테스트 성공!")
            return True
        else:
            logger.error("❌ RGB 카메라 센서 테스트 실패")
            return False
            
    except Exception as e:
        logger.error(f"❌ RGB 카메라 센서 테스트 오류: {e}")
        return False

def main():
    """메인 테스트 함수"""
    logger.info("🚀 Isaac Sim 5.0 RGB 카메라 센서 테스트 시작")
    
    # Create simulation app (headless mode)
    simulation_app = SimulationApp({
        "headless": True,
        "width": 1280,
        "height": 720
    })
    
    try:
        logger.info("✅ Isaac Sim 5.0 SimulationApp 초기화 성공")
        
        # 테스트 씬 설정
        world = setup_test_scene()
        
        # RGB 카메라 센서 테스트
        test_result = test_rgb_camera_sensor()
        
        if test_result:
            logger.info("✅ 모든 테스트 완료 - 성공!")
        else:
            logger.error("❌ 테스트 실패")
            
    except Exception as e:
        logger.error(f"❌ 테스트 실행 오류: {e}")
        
    finally:
        # Isaac Sim 정리
        simulation_app.close()
        logger.info("🔚 Isaac Sim 종료")

if __name__ == "__main__":
    main()