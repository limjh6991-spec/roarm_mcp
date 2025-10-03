#!/usr/bin/env python3
"""
개선된 RGB 카메라 센서 모듈 (전문가 피드백 반영)
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
from typing import Optional, Tuple, Dict, Any, List
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EnhancedRGBCameraSensor:
    """Isaac Sim 5.0 개선된 RGB 카메라 센서 클래스"""
    
    def __init__(self, 
                 camera_path: str = "/World/Sensors/FrontCam_Mount/FrontCam",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0):
        """
        개선된 RGB 카메라 센서 초기화
        
        Args:
            camera_path: 카메라 프림 경로
            resolution: 이미지 해상도 (width, height)
            output_dir: 출력 디렉토리 (선택사항)
            fps: 목표 프레임레이트
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/enhanced_rgb_camera"
        self.fps = fps
        
        # Isaac Sim 모듈들 (런타임에 import)
        self.rep = None
        self.render_product = None
        self.writer = None
        self.annotator = None
        
        # 성능 메트릭
        self.capture_count = 0
        self.last_capture_time = 0.0
        self.frame_warmup_count = 3  # 워밍업 프레임 수
        self.black_frame_threshold = 1.0  # 블랙 프레임 감지 임계값
        
        # 압축 설정
        self.jpeg_quality = 95  # 높은 품질로 설정
        self.png_compression = 6  # PNG 압축 레벨 (0-9)
        
        # 카메라 내부 파라미터 (해상도에 맞게 조정)
        self._calculate_camera_parameters()
        
        logger.info(f"EnhancedRGBCameraSensor 초기화: {camera_path}, 해상도: {resolution}")
    
    def _calculate_camera_parameters(self):
        """카메라 내·외부 파라미터 정합 (16:9 해상도)"""
        width, height = self.resolution
        aspect_ratio = width / height
        
        # 조리개 크기를 해상도 비율에 맞게 조정
        self.horizontal_aperture = 36.0  # mm
        self.vertical_aperture = self.horizontal_aperture / aspect_ratio  # 16:9 = 20.25mm
        self.focal_length = 24.0  # mm
        
        # 픽셀 단위 초점거리 계산
        self.fx = self.focal_length * width / self.horizontal_aperture
        self.fy = self.focal_length * height / self.vertical_aperture
        
        # 주점 (이미지 중심)
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        # 카메라 내부 파라미터 행렬 K
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # 왜곡 계수 (핀홀 카메라 모델이므로 0)
        self.distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        logger.info(f"카메라 파라미터 계산 완료:")
        logger.info(f"  조리개: {self.horizontal_aperture:.1f}x{self.vertical_aperture:.2f}mm")
        logger.info(f"  초점거리: fx={self.fx:.1f}, fy={self.fy:.1f}")
        logger.info(f"  주점: cx={self.cx:.1f}, cy={self.cy:.1f}")
    
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
            
            # RGB Annotator 사용 (성능 최적화)
            self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self.annotator.attach([self.render_product])
            
            # BasicWriter 초기화 (백업용)
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                output_dir=self.output_dir,
                rgb=True,
                camera_params=True
            )
            self.writer.attach([self.render_product])
            
            # 카메라 정보 저장
            self._save_camera_info()
            
            logger.info(f"카메라 센서 초기화 성공: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"카메라 센서 초기화 실패: {e}")
            return False
    
    def _save_camera_info(self):
        """ROS 호환 카메라 정보 저장"""
        camera_info = {
            "image_width": self.resolution[0],
            "image_height": self.resolution[1],
            "camera_name": "isaac_sim_rgb_camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": self.camera_matrix.flatten().tolist()
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": self.distortion_coeffs.tolist()
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": np.eye(3).flatten().tolist()
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": np.column_stack([self.camera_matrix, np.zeros(3)]).flatten().tolist()
            },
            # Isaac Sim 특화 정보
            "isaac_sim": {
                "horizontal_aperture_mm": self.horizontal_aperture,
                "vertical_aperture_mm": self.vertical_aperture,
                "focal_length_mm": self.focal_length,
                "camera_path": self.camera_path,
                "fps": self.fps
            }
        }
        
        camera_info_path = Path(self.output_dir) / "camera_info.json"
        with open(camera_info_path, 'w') as f:
            json.dump(camera_info, f, indent=2)
        
        logger.info(f"카메라 정보 저장 완료: {camera_info_path}")
    
    def _warmup_frames(self, count: int = 3):
        """프레임 워밍업 (블랙 프레임 방지)"""
        try:
            import omni.kit.app
            app = omni.kit.app.get_app()
            
            logger.debug(f"프레임 워밍업 시작 ({count}회)")
            for i in range(count):
                app.update()
                time.sleep(0.01)  # 10ms 대기
                
                if i == count - 1:  # 마지막 워밍업에서 렌더링
                    self.rep.orchestrator.step()
                    if self.writer:
                        self.writer.flush()
                        
            logger.debug("프레임 워밍업 완료")
            
        except Exception as e:
            logger.warning(f"프레임 워밍업 중 오류: {e}")
    
    def _check_black_frame(self, image: np.ndarray) -> bool:
        """블랙 프레임 감지"""
        if image is None or image.size == 0:
            return True
            
        mean_val = image.mean()
        std_val = image.std()
        
        is_black = mean_val < self.black_frame_threshold and std_val < self.black_frame_threshold
        
        if is_black:
            logger.warning(f"⚠️ 블랙 프레임 감지: mean={mean_val:.2f}, std={std_val:.2f}")
        else:
            logger.debug(f"프레임 통계: mean={mean_val:.1f}, std={std_val:.1f}")
            
        return is_black
    
    def capture_rgb_image_direct(self) -> Optional[np.ndarray]:
        """
        Annotator를 통한 직접 RGB 이미지 캡처 (성능 최적화)
        
        Returns:
            RGB 이미지 배열 (H, W, 3) 또는 None
        """
        if not self.annotator:
            logger.error("Annotator가 초기화되지 않음")
            return None
        
        try:
            start_time = time.time()
            
            # 워밍업 (첫 캡처시에만)
            if self.capture_count == 0:
                self._warmup_frames(self.frame_warmup_count)
            
            # Annotator를 통한 직접 데이터 획득
            self.rep.orchestrator.step()
            rgb_data = self.annotator.get_data()
            
            if rgb_data is None:
                logger.warning("Annotator에서 데이터를 가져올 수 없음")
                return None
            
            # numpy 배열로 변환
            if hasattr(rgb_data, 'cpu'):
                # GPU tensor인 경우
                rgb_array = rgb_data.cpu().numpy()
            else:
                rgb_array = np.array(rgb_data)
            
            # 데이터 타입 및 범위 정규화
            if rgb_array.dtype == np.float32 or rgb_array.dtype == np.float64:
                rgb_array = (rgb_array * 255).astype(np.uint8)
            
            # RGBA에서 RGB로 변환 (필요시)
            if rgb_array.shape[-1] == 4:
                rgb_array = rgb_array[..., :3]
            
            # 블랙 프레임 체크
            if self._check_black_frame(rgb_array):
                # 재시도
                logger.info("블랙 프레임 감지, 재시도...")
                self._warmup_frames(1)
                self.rep.orchestrator.step()
                rgb_data = self.annotator.get_data()
                
                if rgb_data is not None:
                    if hasattr(rgb_data, 'cpu'):
                        rgb_array = rgb_data.cpu().numpy()
                    else:
                        rgb_array = np.array(rgb_data)
                    
                    if rgb_array.dtype == np.float32 or rgb_array.dtype == np.float64:
                        rgb_array = (rgb_array * 255).astype(np.uint8)
                    
                    if rgb_array.shape[-1] == 4:
                        rgb_array = rgb_array[..., :3]
            
            # 성능 메트릭 업데이트
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"RGB 이미지 직접 캡처 성공: {rgb_array.shape}, 시간: {self.last_capture_time:.3f}s")
            return rgb_array
            
        except Exception as e:
            logger.error(f"RGB 이미지 직접 캡처 실패: {e}")
            return None
    
    def capture_rgb_image_file(self) -> Optional[np.ndarray]:
        """
        파일 기반 RGB 이미지 캡처 (백업 방법)
        
        Returns:
            RGB 이미지 배열 (H, W, 3) 또는 None
        """
        if not self.rep or not self.render_product:
            logger.error("카메라 센서가 초기화되지 않음")
            return None
        
        try:
            start_time = time.time()
            
            # 워밍업 (첫 캡처시에만)
            if self.capture_count == 0:
                self._warmup_frames(self.frame_warmup_count)
            
            # Replicator 렌더링 단계 실행
            self.rep.orchestrator.step()
            if self.writer:
                self.writer.flush()  # 즉시 디스크에 기록
            
            # 최신 생성된 RGB 파일 찾기 (재시도 로직)
            max_retries = 3
            rgb_files = []
            
            for retry in range(max_retries):
                rgb_files = list(Path(self.output_dir).glob("rgb_*.png"))
                if rgb_files:
                    break
                time.sleep(0.05)  # 50ms 대기 후 재시도
            
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
            
            # 블랙 프레임 체크
            if self._check_black_frame(rgb_image):
                logger.warning(f"블랙 프레임 파일 크기: {latest_file.stat().st_size} bytes")
            
            # 성능 메트릭 업데이트
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"RGB 이미지 파일 캡처 성공: {rgb_image.shape}, 시간: {self.last_capture_time:.3f}s")
            return rgb_image
            
        except Exception as e:
            logger.error(f"RGB 이미지 파일 캡처 실패: {e}")
            return None
    
    def capture_rgb_image(self, method: str = "direct") -> Optional[np.ndarray]:
        """
        RGB 이미지 캡처 (메소드 선택 가능)
        
        Args:
            method: "direct" (Annotator) 또는 "file" (파일 기반)
            
        Returns:
            RGB 이미지 배열 (H, W, 3) 또는 None
        """
        if method == "direct":
            return self.capture_rgb_image_direct()
        elif method == "file":
            return self.capture_rgb_image_file()
        else:
            logger.error(f"지원하지 않는 캡처 방법: {method}")
            return None
    
    def compress_image(self, 
                      image: np.ndarray, 
                      format: str = "JPEG", 
                      quality: int = None) -> Optional[bytes]:
        """
        개선된 이미지 압축
        
        Args:
            image: RGB 이미지 배열
            format: 압축 포맷 ("JPEG" 또는 "PNG")
            quality: JPEG 품질 (1-100) 또는 PNG 압축 레벨 (0-9)
            
        Returns:
            압축된 이미지 바이트 또는 None
        """
        try:
            # BGR로 변환 (OpenCV 요구사항)
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if format.upper() == "JPEG":
                if quality is None:
                    quality = self.jpeg_quality
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
                success, encoded_img = cv2.imencode('.jpg', bgr_image, encode_params)
                
            elif format.upper() == "PNG":
                if quality is None:
                    quality = self.png_compression
                encode_params = [cv2.IMWRITE_PNG_COMPRESSION, quality]
                success, encoded_img = cv2.imencode('.png', bgr_image, encode_params)
                
            else:
                logger.error(f"지원하지 않는 이미지 포맷: {format}")
                return None
            
            if not success:
                logger.error("이미지 인코딩 실패")
                return None
                
            compressed_bytes = encoded_img.tobytes()
            
            # 압축 효율 로깅
            original_size = image.nbytes
            compressed_size = len(compressed_bytes)
            compression_ratio = original_size / compressed_size if compressed_size > 0 else 0
            
            logger.debug(f"압축 완료: {format} {original_size}→{compressed_size} bytes "
                        f"(비율: {compression_ratio:.1f}x)")
            
            return compressed_bytes
            
        except Exception as e:
            logger.error(f"이미지 압축 실패: {e}")
            return None
    
    def encode_base64(self, image_bytes: bytes) -> str:
        """이미지 바이트를 Base64로 인코딩"""
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def capture_and_encode_image(self, 
                               format: str = "JPEG",
                               quality: int = None,
                               base64_encode: bool = True,
                               method: str = "direct") -> Optional[Dict[str, Any]]:
        """
        개선된 이미지 캡처, 압축 및 인코딩
        
        Args:
            format: 압축 포맷
            quality: 압축 품질
            base64_encode: Base64 인코딩 여부
            method: 캡처 방법 ("direct" 또는 "file")
            
        Returns:
            이미지 데이터 딕셔너리 또는 None
        """
        try:
            # RGB 이미지 캡처
            rgb_image = self.capture_rgb_image(method)
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
                "capture_time_ms": self.last_capture_time * 1000,
                "capture_method": method,
                "camera_info": {
                    "fx": self.fx,
                    "fy": self.fy,
                    "cx": self.cx,
                    "cy": self.cy,
                    "camera_matrix": self.camera_matrix.tolist(),
                    "distortion_coeffs": self.distortion_coeffs.tolist()
                }
            }
            
            if base64_encode:
                result["image_base64"] = self.encode_base64(compressed_bytes)
            else:
                result["image_bytes"] = compressed_bytes
            
            logger.info(f"이미지 캡처 완료 ({method}): {result['format']}, "
                       f"{result['size_bytes']} bytes, {result['capture_time_ms']:.1f}ms")
            return result
            
        except Exception as e:
            logger.error(f"이미지 캡처 및 인코딩 실패: {e}")
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """확장된 성능 메트릭"""
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "target_fps": self.fps,
            "actual_fps": 1.0 / self.last_capture_time if self.last_capture_time > 0 else 0,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "camera_parameters": {
                "fx": self.fx,
                "fy": self.fy,
                "cx": self.cx,
                "cy": self.cy,
                "horizontal_aperture_mm": self.horizontal_aperture,
                "vertical_aperture_mm": self.vertical_aperture,
                "focal_length_mm": self.focal_length
            },
            "output_directory": self.output_dir,
            "compression_settings": {
                "jpeg_quality": self.jpeg_quality,
                "png_compression": self.png_compression
            }
        }
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.annotator:
                self.annotator.detach()
            if self.writer:
                self.writer.detach()
            logger.info("Enhanced RGB 카메라 센서 리소스 정리 완료")
        except Exception as e:
            logger.error(f"리소스 정리 중 오류: {e}")


def test_enhanced_rgb_camera_sensor():
    """개선된 RGB 카메라 센서 테스트 함수"""
    print("🎥 개선된 RGB 카메라 센서 테스트 시작")
    
    try:
        # Isaac Sim 초기화
        from isaacsim import SimulationApp
        
        # 성능 최적화 설정
        simulation_app = SimulationApp({
            "headless": True,
            "width": 1280,
            "height": 720,
            "renderer": "RayTracedLighting",  # 더 나은 조명
            "anti_aliasing": 3,  # 안티앨리어싱
            "samples_per_pixel": 64,  # 샘플링 품질
            "denoiser": True,  # 노이즈 제거
            "subdiv_refinement_level": 2,
            "enable_scene_query_support": False,  # 불필요한 기능 비활성화
            "enable_livestream": False,
            "kit_settings": {
                "/app/fastShutdown": False,  # 안전한 종료
                "/rtx/materialDb/syncLoads": True,
                "/rtx/hydra/materialSyncLoads": True,
                "/omni.kit.plugin/syncUsdLoads": True
            }
        })
        
        print("✅ Isaac Sim 초기화 완료 (성능 최적화 적용)")
        
        # USD 환경 설정 (개선된 조명)
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
        
        # 개선된 조명 설정 (더 밝고 현실적)
        lights_mount = stage.DefinePrim(Sdf.Path("/World/Lights"), "Xform")
        
        # 주 조명 (더 밝게)
        distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/DistantLight"))
        distant_light.CreateIntensityAttr(10000.0)  # 강한 직사광
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        distant_light.CreateAngleAttr(0.5)  # 태양 크기
        
        # 환경광 (더 밝게)
        dome_light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/DomeLight"))
        dome_light.CreateIntensityAttr(3000.0)  # 강한 환경광
        dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.95, 1.0))  # 약간 푸른 하늘색
        
        print("  - 개선된 조명 시스템 설정 완료")
        
        # 재질이 있는 바닥 평면
        ground_geom = UsdGeom.Cube.Define(stage, Sdf.Path("/World/GroundPlane"))
        ground_geom.CreateSizeAttr(1.0)
        ground_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))
        ground_geom.AddScaleOp().Set(Gf.Vec3f(20, 20, 1))
        
        # 다양한 색상의 테스트 객체들
        colors = [
            (Gf.Vec3f(1, 0, 0), "RedCube", Gf.Vec3f(-2, 0, 1)),
            (Gf.Vec3f(0, 1, 0), "GreenSphere", Gf.Vec3f(0, 0, 1.5)),
            (Gf.Vec3f(0, 0, 1), "BlueCylinder", Gf.Vec3f(2, 0, 1)),
            (Gf.Vec3f(1, 1, 0), "YellowCube", Gf.Vec3f(-1, 2, 1)),
            (Gf.Vec3f(1, 0, 1), "MagentaCube", Gf.Vec3f(1, -2, 1))
        ]
        
        for color, name, pos in colors:
            if "Sphere" in name:
                obj = UsdGeom.Sphere.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(0.5)
            elif "Cylinder" in name:
                obj = UsdGeom.Cylinder.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(0.3)
                obj.CreateHeightAttr(1.0)
            else:
                obj = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateSizeAttr(1.0)
            
            obj.AddTranslateOp().Set(pos)
            obj.CreateDisplayColorAttr([color])
        
        print("  - 다채로운 테스트 환경 생성 완료")
        
        # 개선된 카메라 마운트 및 카메라 생성
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(-8.0, 0.0, 3.0))  # 더 좋은 시점
        cam_mount_xform.SetRotate(Gf.Vec3f(0, 10, 0))  # 약간 아래로
        
        # 개선된 카메라 설정 (해상도/조리개 정합)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        
        # 16:9 해상도에 맞는 조리개 설정
        camera_prim.CreateHorizontalApertureAttr(36.0)  # mm
        camera_prim.CreateVerticalApertureAttr(20.25)   # 36 * (9/16) = 20.25mm
        camera_prim.CreateFocalLengthAttr(24.0)         # mm
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
        camera_prim.CreateFStopAttr(2.8)  # 조리개 값
        
        print("  - 개선된 카메라 설정 완료 (해상도/조리개 정합)")
        
        print("✅ USD 환경 설정 완료")
        
        # Enhanced RGB 카메라 센서 생성 및 초기화
        camera_sensor = EnhancedRGBCameraSensor(
            camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
            resolution=(1280, 720),
            output_dir="/tmp/enhanced_rgb_camera_test",
            fps=30.0
        )
        
        if not camera_sensor.initialize():
            print("❌ 카메라 센서 초기화 실패")
            return False
        
        print("✅ Enhanced RGB 카메라 센서 초기화 완료")
        
        # 시뮬레이션 업데이트
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        print("🔄 시뮬레이션 환경 준비 중...")
        for i in range(20):
            app.update()
            if i % 5 == 0:
                print(f"  - 프레임 {i+1}/20 업데이트")
        
        print("✅ 시뮬레이션 환경 준비 완료")
        
        # 이미지 캡처 및 압축 테스트 (두 방법 비교)
        print("\n📷 개선된 이미지 캡처 테스트...")
        
        test_cases = [
            {"method": "direct", "format": "JPEG", "quality": 95},
            {"method": "file", "format": "JPEG", "quality": 85},
            {"method": "direct", "format": "PNG", "quality": 6},
        ]
        
        for i, test_case in enumerate(test_cases):
            print(f"\n--- 테스트 {i+1}/{len(test_cases)}: {test_case['method']} {test_case['format']} ---")
            
            result = camera_sensor.capture_and_encode_image(
                format=test_case['format'],
                quality=test_case['quality'],
                base64_encode=True,
                method=test_case['method']
            )
            
            if result:
                print(f"  ✅ {test_case['format']}: {result['size_bytes']} bytes, "
                      f"{result['capture_time_ms']:.1f}ms")
                print(f"  📐 해상도: {result['resolution']}")
                print(f"  🎯 카메라 정보: fx={result['camera_info']['fx']:.1f}, "
                      f"fy={result['camera_info']['fy']:.1f}")
                if 'image_base64' in result:
                    print(f"  📝 Base64 길이: {len(result['image_base64'])}")
            else:
                print(f"  ❌ {test_case['method']} {test_case['format']} 캡처 실패")
            
            time.sleep(0.1)
        
        # 성능 메트릭 출력
        metrics = camera_sensor.get_performance_metrics()
        print(f"\n📊 성능 메트릭:")
        print(f"  총 캡처 횟수: {metrics['total_captures']}")
        print(f"  마지막 캡처 시간: {metrics['last_capture_time_ms']:.1f}ms")
        print(f"  실제 FPS: {metrics['actual_fps']:.1f} (목표: {metrics['target_fps']})")
        print(f"  카메라 파라미터: fx={metrics['camera_parameters']['fx']:.1f}, "
              f"fy={metrics['camera_parameters']['fy']:.1f}")
        print(f"  조리개: {metrics['camera_parameters']['horizontal_aperture_mm']:.1f}x"
              f"{metrics['camera_parameters']['vertical_aperture_mm']:.2f}mm")
        
        # 캡처된 파일들 확인
        output_path = Path(camera_sensor.output_dir)
        if output_path.exists():
            files = list(output_path.glob("*"))
            print(f"\n📁 생성된 파일들 ({len(files)}개):")
            for file_path in sorted(files)[:10]:  # 최대 10개 표시
                size = file_path.stat().st_size
                print(f"  - {file_path.name}: {size:,} bytes")
        
        # 리소스 정리
        camera_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ 개선된 RGB 카메라 센서 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_enhanced_rgb_camera_sensor()
    sys.exit(0 if success else 1)