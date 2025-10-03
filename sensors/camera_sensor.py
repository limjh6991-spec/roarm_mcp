#!/usr/bin/env python3
"""
RGB Camera Sensor Module for Isaac Sim 5.0
Isaac Sim 5.0 RGB 카메라 센서 모듈

이 모듈은 Isaac Sim 5.0의 Camera API를 사용하여
RGB 이미지 캡처 및 실시간 스트리밍 기능을 제공합니다.
"""

import numpy as np
import torch
import cv2
import base64
import json
import asyncio
import logging
from typing import Optional, Tuple, Dict, Any
import time

# Isaac Sim 5.0 imports
try:
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.prims import create_prim
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Usd, UsdGeom, Gf
    ISAAC_SIM_AVAILABLE = True
except ImportError as e:
    # Fallback to older Isaac Sim import paths
    try:
        from omni.isaac.sensor import Camera
        from omni.isaac.core.utils.prims import create_prim
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import Usd, UsdGeom, Gf
        ISAAC_SIM_AVAILABLE = True
        logging.info("Using legacy Isaac Sim import paths")
    except ImportError as e2:
        logging.warning(f"Isaac Sim modules not available: {e}, {e2}")
        ISAAC_SIM_AVAILABLE = False

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RGBCameraSensor:
    """Isaac Sim 5.0 RGB 카메라 센서 클래스"""
    
    def __init__(self, 
                 prim_path: str = "/World/RGBCamera",
                 position: np.ndarray = np.array([2.0, 2.0, 2.0]),
                 target: np.ndarray = np.array([0.0, 0.0, 0.0]),
                 resolution: Tuple[int, int] = (1280, 720),
                 frequency: float = 30.0):
        """
        RGB 카메라 센서 초기화
        
        Args:
            prim_path: USD 씬에서의 카메라 경로
            position: 카메라 3D 위치 (x, y, z)
            target: 카메라가 바라볼 타겟 위치
            resolution: 이미지 해상도 (width, height)
            frequency: 캡처 주파수 (FPS)
        """
        self.prim_path = prim_path
        self.position = position
        self.target = target
        self.resolution = resolution
        self.frequency = frequency
        self.camera = None
        self.is_initialized = False
        
        # 성능 모니터링
        self.capture_count = 0
        self.last_capture_time = 0
        self.fps_history = []
        
        logger.info(f"🎥 RGBCameraSensor 초기화: {prim_path}")
        logger.info(f"   위치: {position}, 타겟: {target}")
        logger.info(f"   해상도: {resolution}, 주파수: {frequency}Hz")
        
    def initialize(self) -> bool:
        """
        카메라 센서 초기화 및 Isaac Sim 씬에 추가
        
        Returns:
            bool: 초기화 성공 여부
        """
        if not ISAAC_SIM_AVAILABLE:
            logger.error("❌ Isaac Sim 모듈을 사용할 수 없습니다")
            return False
            
        try:
            logger.info("🎬 RGB 카메라 센서 초기화 중...")
            
            # Isaac Sim Camera 생성
            self.camera = Camera(
                prim_path=self.prim_path,
                position=self.position,
                target=self.target,
                resolution=self.resolution,
                frequency=self.frequency
            )
            
            # 카메라 속성 설정
            self._configure_camera_properties()
            
            self.is_initialized = True
            logger.info("✅ RGB 카메라 센서 초기화 완료")
            return True
            
        except Exception as e:
            logger.error(f"❌ RGB 카메라 초기화 실패: {e}")
            return False
            
    def _configure_camera_properties(self):
        """카메라 세부 속성 설정"""
        if not self.camera:
            return
            
        try:
            # 카메라 파라미터 설정
            camera_prim = get_current_stage().GetPrimAtPath(self.prim_path)
            if camera_prim:
                # FOV 설정 (시야각)
                camera_api = UsdGeom.Camera(camera_prim)
                camera_api.GetFocalLengthAttr().Set(24.0)  # 24mm 렌즈
                camera_api.GetFStopAttr().Set(2.8)  # F-stop
                
                logger.info("🔧 카메라 속성 설정 완료")
                
        except Exception as e:
            logger.warning(f"⚠️ 카메라 속성 설정 부분 실패: {e}")
            
    def capture_rgb_image(self) -> Optional[np.ndarray]:
        """
        RGB 이미지 캡처
        
        Returns:
            np.ndarray: RGB 이미지 데이터 (H, W, 3) 또는 None
        """
        if not self.is_initialized or not self.camera:
            logger.error("❌ 카메라가 초기화되지 않았습니다")
            return None
            
        try:
            # Isaac Sim에서 RGBA 이미지 캡처
            rgba_data = self.camera.get_rgba()
            
            if rgba_data is None:
                logger.warning("⚠️ 카메라에서 이미지를 가져올 수 없습니다")
                return None
                
            # RGBA에서 RGB만 추출 (Alpha 채널 제거)
            rgb_data = rgba_data[:, :, :3]
            
            # 성능 모니터링 업데이트
            self._update_performance_metrics()
            
            logger.debug(f"📸 RGB 이미지 캡처 완료: {rgb_data.shape}")
            return rgb_data
            
        except Exception as e:
            logger.error(f"❌ RGB 이미지 캡처 실패: {e}")
            return None
            
    def _update_performance_metrics(self):
        """성능 메트릭 업데이트"""
        current_time = time.time()
        
        if self.last_capture_time > 0:
            fps = 1.0 / (current_time - self.last_capture_time)
            self.fps_history.append(fps)
            
            # 최근 10개 프레임의 평균 FPS 유지
            if len(self.fps_history) > 10:
                self.fps_history.pop(0)
                
        self.last_capture_time = current_time
        self.capture_count += 1
        
    def get_performance_metrics(self) -> Dict[str, Any]:
        """성능 메트릭 반환"""
        if not self.fps_history:
            return {"fps": 0.0, "capture_count": self.capture_count}
            
        avg_fps = sum(self.fps_history) / len(self.fps_history)
        return {
            "fps": round(avg_fps, 2),
            "capture_count": self.capture_count,
            "target_fps": self.frequency,
            "fps_stability": round(np.std(self.fps_history), 2)
        }
        
    def capture_and_encode_image(self, 
                               quality: int = 85,
                               format: str = "JPEG") -> Optional[str]:
        """
        RGB 이미지 캡처 및 Base64 인코딩
        
        Args:
            quality: JPEG 압축 품질 (1-100)
            format: 이미지 포맷 ("JPEG" 또는 "PNG")
            
        Returns:
            str: Base64 인코딩된 이미지 문자열 또는 None
        """
        # RGB 이미지 캡처
        rgb_image = self.capture_rgb_image()
        if rgb_image is None:
            return None
            
        try:
            # OpenCV 형식으로 변환 (BGR)
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
            # 이미지 압축
            if format.upper() == "JPEG":
                encode_param = [cv2.IMWRITE_JPEG_QUALITY, quality]
                _, buffer = cv2.imencode('.jpg', bgr_image, encode_param)
            elif format.upper() == "PNG":
                encode_param = [cv2.IMWRITE_PNG_COMPRESSION, 9]
                _, buffer = cv2.imencode('.png', bgr_image, encode_param)
            else:
                logger.error(f"❌ 지원하지 않는 이미지 포맷: {format}")
                return None
                
            # Base64 인코딩
            encoded_image = base64.b64encode(buffer).decode('utf-8')
            
            logger.debug(f"🔐 이미지 인코딩 완료: {len(encoded_image)} 문자")
            return encoded_image
            
        except Exception as e:
            logger.error(f"❌ 이미지 인코딩 실패: {e}")
            return None
            
    def get_camera_info(self) -> Dict[str, Any]:
        """카메라 정보 반환"""
        return {
            "prim_path": self.prim_path,
            "position": self.position.tolist(),
            "target": self.target.tolist(),
            "resolution": self.resolution,
            "frequency": self.frequency,
            "is_initialized": self.is_initialized,
            "performance": self.get_performance_metrics()
        }
        
    def update_position(self, 
                       position: Optional[np.ndarray] = None,
                       target: Optional[np.ndarray] = None):
        """
        카메라 위치 업데이트
        
        Args:
            position: 새로운 카메라 위치
            target: 새로운 타겟 위치
        """
        if not self.is_initialized or not self.camera:
            logger.warning("⚠️ 카메라가 초기화되지 않아 위치를 업데이트할 수 없습니다")
            return
            
        try:
            if position is not None:
                self.position = position
                self.camera.set_world_pose(position=position)
                logger.info(f"📍 카메라 위치 업데이트: {position}")
                
            if target is not None:
                self.target = target
                # 타겟 방향으로 카메라 회전
                direction = target - self.position
                direction = direction / np.linalg.norm(direction)
                # TODO: 회전 행렬 계산 및 적용
                logger.info(f"🎯 카메라 타겟 업데이트: {target}")
                
        except Exception as e:
            logger.error(f"❌ 카메라 위치 업데이트 실패: {e}")
            
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.camera:
                # Isaac Sim 카메라 정리
                self.camera = None
                
            self.is_initialized = False
            logger.info("🧹 RGB 카메라 센서 정리 완료")
            
        except Exception as e:
            logger.error(f"❌ 카메라 정리 실패: {e}")
            
    def __enter__(self):
        """Context manager 진입"""
        if self.initialize():
            return self
        else:
            raise RuntimeError("카메라 초기화 실패")
            
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager 종료"""
        self.cleanup()

# 편의 함수들
def create_rgb_camera(prim_path: str = "/World/RGBCamera",
                     position: Tuple[float, float, float] = (2.0, 2.0, 2.0),
                     target: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                     resolution: Tuple[int, int] = (1280, 720),
                     frequency: float = 30.0) -> RGBCameraSensor:
    """
    RGB 카메라 센서 생성 편의 함수
    
    Returns:
        RGBCameraSensor: 초기화된 RGB 카메라 센서 인스턴스
    """
    camera = RGBCameraSensor(
        prim_path=prim_path,
        position=np.array(position),
        target=np.array(target),
        resolution=resolution,
        frequency=frequency
    )
    
    return camera

if __name__ == "__main__":
    # 기본 테스트 코드
    logger.info("🧪 RGB 카메라 센서 모듈 테스트")
    
    if ISAAC_SIM_AVAILABLE:
        logger.info("✅ Isaac Sim 모듈 사용 가능")
    else:
        logger.warning("⚠️ Isaac Sim 모듈 없이 테스트 (실제 카메라 기능 제한)")
        
    # 카메라 센서 생성 테스트
    camera_sensor = create_rgb_camera()
    camera_info = camera_sensor.get_camera_info()
    
    logger.info(f"📋 카메라 정보: {json.dumps(camera_info, indent=2)}")
    logger.info("🎉 RGB 카메라 센서 모듈 테스트 완료")