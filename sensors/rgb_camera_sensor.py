#!/usr/bin/env python3
"""
RGB 카메라 센서 모듈 (기본 버전)
Isaac Sim 5.0 Camera API를 사용한 실시간 RGB 이미지 캡처 및 압축/전송
"""

import os
import sys
import base64
import json
import time
import cv2
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, Dict, Any
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RGBCameraSensor:
    """Isaac Sim 5.0 RGB 카메라 센서 클래스 (기본 버전)"""
    
    def __init__(self, 
                 camera_path: str = "/World/Sensors/FrontCam_Mount/FrontCam",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None):
        """
        RGB 카메라 센서 초기화
        
        Args:
            camera_path: 카메라 프림 경로
            resolution: 이미지 해상도 (width, height)
            output_dir: 출력 디렉토리 (선택사항)
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/rgb_camera"
        
        # Isaac Sim 모듈들 (런타임에 import)
        self.rep = None
        self.render_product = None
        self.writer = None
        
        # 성능 메트릭
        self.capture_count = 0
        self.last_capture_time = 0.0
        
        logger.info(f"RGBCameraSensor 초기화: {camera_path}, 해상도: {resolution}")

import os
import sys
import base64
import json
import time
import cv2
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, Dict, Any
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RGBCameraSensor:
    """Isaac Sim 5.0 RGB 카메라 센서 클래스"""
    
    def __init__(self, 
                 camera_path: str = "/World/Sensors/FrontCam_Mount/FrontCam",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None):
        """
        RGB 카메라 센서 초기화
        
        Args:
            camera_path: 카메라 프림 경로
            resolution: 이미지 해상도 (width, height)
            output_dir: 출력 디렉토리 (선택사항)
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/rgb_camera"
        
        # Isaac Sim 모듈들 (런타임에 import)
        self.rep = None
        self.render_product = None
        self.writer = None
        
        # 성능 메트릭
        self.capture_count = 0
        self.last_capture_time = 0.0
        
        # 압축 설정
        self.jpeg_quality = 85
        
        logger.info(f"RGBCameraSensor 초기화: {camera_path}, 해상도: {resolution}")
    
    def initialize(self) -> bool:
        """
        카메라 센서 초기화 (Isaac Sim 환경에서 호출)
        
        Returns:
            초기화 성공 여부
        """
        try:
            # Isaac Sim 모듈 import
            import omni.replicator.core as rep
            self.rep = rep
            
            # 출력 디렉토리 생성
            os.makedirs(self.output_dir, exist_ok=True)
            
            # 렌더 제품 생성
            self.render_product = rep.create.render_product(
                self.camera_path, 
                self.resolution
            )
            
            # BasicWriter 초기화 (RGB 이미지 캡처용)
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                output_dir=self.output_dir,
                rgb=True,
                camera_params=True
            )
            self.writer.attach([self.render_product])
            
            logger.info(f"카메라 센서 초기화 성공: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"카메라 센서 초기화 실패: {e}")
            return False
    
    def capture_rgb_image(self) -> Optional[np.ndarray]:
        """
        RGB 이미지 캡처
        
        Returns:
            RGB 이미지 배열 (H, W, 3) 또는 None
        """
        if not self.rep or not self.render_product:
            logger.error("카메라 센서가 초기화되지 않음")
            return None
        
        try:
            start_time = time.time()
            
            # Replicator 렌더링 단계 실행
            self.rep.orchestrator.step()
            
            # 최신 생성된 RGB 파일 찾기
            rgb_files = list(Path(self.output_dir).glob("rgb_*.png"))
            if not rgb_files:
                logger.warning("RGB 이미지 파일을 찾을 수 없음")
                return None
            
            # 가장 최근 파일 선택
            latest_file = max(rgb_files, key=lambda x: x.stat().st_mtime)
            
            # OpenCV로 이미지 로드
            image = cv2.imread(str(latest_file))
            if image is None:
                logger.error(f"이미지 로드 실패: {latest_file}")
                return None
            
            # BGR에서 RGB로 변환
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # 성능 메트릭 업데이트
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"RGB 이미지 캡처 성공: {rgb_image.shape}, 시간: {self.last_capture_time:.3f}s")
            return rgb_image
            
        except Exception as e:
            logger.error(f"RGB 이미지 캡처 실패: {e}")
            return None
    
    def compress_image(self, 
                      image: np.ndarray, 
                      format: str = "JPEG", 
                      quality: int = None) -> Optional[bytes]:
        """
        이미지 압축
        
        Args:
            image: RGB 이미지 배열
            format: 압축 포맷 ("JPEG" 또는 "PNG")
            quality: JPEG 품질 (1-100)
            
        Returns:
            압축된 이미지 바이트 또는 None
        """
        try:
            if quality is None:
                quality = self.jpeg_quality
                
            # BGR로 변환 (OpenCV 요구사항)
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if format.upper() == "JPEG":
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
                success, encoded_img = cv2.imencode('.jpg', bgr_image, encode_params)
            elif format.upper() == "PNG":
                encode_params = [cv2.IMWRITE_PNG_COMPRESSION, 9]
                success, encoded_img = cv2.imencode('.png', bgr_image, encode_params)
            else:
                logger.error(f"지원하지 않는 이미지 포맷: {format}")
                return None
            
            if not success:
                logger.error("이미지 인코딩 실패")
                return None
                
            return encoded_img.tobytes()
            
        except Exception as e:
            logger.error(f"이미지 압축 실패: {e}")
            return None
    
    def encode_base64(self, image_bytes: bytes) -> str:
        """
        이미지 바이트를 Base64로 인코딩
        
        Args:
            image_bytes: 압축된 이미지 바이트
            
        Returns:
            Base64 인코딩된 문자열
        """
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def capture_and_encode_image(self, 
                               format: str = "JPEG",
                               quality: int = None,
                               base64_encode: bool = True) -> Optional[Dict[str, Any]]:
        """
        이미지 캡처, 압축 및 인코딩을 한 번에 수행
        
        Args:
            format: 압축 포맷
            quality: JPEG 품질
            base64_encode: Base64 인코딩 여부
            
        Returns:
            이미지 데이터 딕셔너리 또는 None
        """
        try:
            # RGB 이미지 캡처
            rgb_image = self.capture_rgb_image()
            if rgb_image is None:
                return None
            
            # 이미지 압축
            compressed_bytes = self.compress_image(rgb_image, format, quality)
            if compressed_bytes is None:
                return None
            
            # 결과 딕셔너리 생성
            result = {
                "timestamp": time.time(),
                "resolution": list(rgb_image.shape[:2][::-1]),  # [width, height]
                "format": format.lower(),
                "size_bytes": len(compressed_bytes),
                "capture_time_ms": self.last_capture_time * 1000
            }
            
            if base64_encode:
                result["image_base64"] = self.encode_base64(compressed_bytes)
            else:
                result["image_bytes"] = compressed_bytes
            
            logger.info(f"이미지 캡처 완료: {result['format']}, {result['size_bytes']} bytes")
            return result
            
        except Exception as e:
            logger.error(f"이미지 캡처 및 인코딩 실패: {e}")
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        성능 메트릭 반환
        
        Returns:
            성능 메트릭 딕셔너리
        """
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "output_directory": self.output_dir
        }
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.writer:
                self.writer.detach()
            logger.info("RGB 카메라 센서 리소스 정리 완료")
        except Exception as e:
            logger.error(f"리소스 정리 중 오류: {e}")


def test_rgb_camera_sensor():
    """RGB 카메라 센서 테스트 함수"""
    print("🎥 RGB 카메라 센서 테스트 시작")
    
    try:
        # Isaac Sim 초기화
        from isaacsim import SimulationApp
        
        simulation_app = SimulationApp({
            "headless": True,
            "width": 1280,
            "height": 720
        })
        
        print("✅ Isaac Sim 초기화 완료")
        
        # USD 환경 설정
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            stage = Usd.Stage.CreateNew("memory_stage.usd")
            usd_context.attach_stage(stage)
        
        stage.SetMetadata("metersPerUnit", 1.0)
        
        # 기본 환경 생성
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        # 조명 설정
        lights_mount = stage.DefinePrim(Sdf.Path("/World/Lights"), "Xform")
        distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/DistantLight"))
        distant_light.CreateIntensityAttr(5000.0)
        
        # 카메라 마운트 및 카메라 생성
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(-5.0, 0.0, 2.0))
        
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        camera_prim.CreateHorizontalApertureAttr(36.0)
        camera_prim.CreateVerticalApertureAttr(24.0)
        camera_prim.CreateFocalLengthAttr(24.0)
        
        # 테스트 객체 추가
        test_cube = UsdGeom.Cube.Define(stage, Sdf.Path("/World/TestCube"))
        test_cube.AddTranslateOp().Set(Gf.Vec3f(0, 0, 1))
        
        print("✅ USD 환경 설정 완료")
        
        # RGB 카메라 센서 생성 및 초기화
        camera_sensor = RGBCameraSensor(
            camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
            resolution=(1280, 720),
            output_dir="/tmp/rgb_camera_test"
        )
        
        if not camera_sensor.initialize():
            print("❌ 카메라 센서 초기화 실패")
            return False
        
        print("✅ RGB 카메라 센서 초기화 완료")
        
        # 시뮬레이션 업데이트
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        for i in range(10):
            app.update()
        
        print("✅ 시뮬레이션 환경 준비 완료")
        
        # 이미지 캡처 및 압축 테스트
        print("\n📷 이미지 캡처 테스트...")
        
        for i in range(3):
            print(f"\n--- 캡처 {i+1}/3 ---")
            
            # JPEG 압축 테스트
            result_jpeg = camera_sensor.capture_and_encode_image(
                format="JPEG", 
                quality=85,
                base64_encode=True
            )
            
            if result_jpeg:
                print(f"  JPEG: {result_jpeg['size_bytes']} bytes, "
                      f"{result_jpeg['capture_time_ms']:.1f}ms")
                print(f"  Base64 길이: {len(result_jpeg['image_base64'])}")
            
            # PNG 압축 테스트
            result_png = camera_sensor.capture_and_encode_image(
                format="PNG",
                base64_encode=False
            )
            
            if result_png:
                print(f"  PNG: {result_png['size_bytes']} bytes, "
                      f"{result_png['capture_time_ms']:.1f}ms")
            
            time.sleep(0.1)
        
        # 성능 메트릭 출력
        metrics = camera_sensor.get_performance_metrics()
        print(f"\n📊 성능 메트릭:")
        print(f"  총 캡처 횟수: {metrics['total_captures']}")
        print(f"  평균 캡처 시간: {metrics['last_capture_time_ms']:.1f}ms")
        print(f"  카메라 경로: {metrics['camera_path']}")
        print(f"  해상도: {metrics['resolution']}")
        
        # 리소스 정리
        camera_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ RGB 카메라 센서 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_rgb_camera_sensor()
    sys.exit(0 if success else 1)