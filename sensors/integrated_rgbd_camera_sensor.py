#!/usr/bin/env python3
"""
통합 RGB-D 카메라 센서 시스템
Isaac Sim 5.0에서 RGB와 Depth를 동기화하여 캡처하는 시스템
"""

import os
import sys
import base64
import json
import time
import threading
import queue
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List, Union
import logging
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, asdict

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class CameraCaptureResult:
    """카메라 캡처 결과 데이터 클래스"""
    timestamp: float
    sensor_type: str  # "rgb" 또는 "depth"
    resolution: Tuple[int, int]
    format: str
    size_bytes: int
    capture_time_ms: float
    capture_method: str
    success: bool
    error_message: Optional[str] = None
    
    # 센서별 추가 데이터
    rgb_data: Optional[Dict[str, Any]] = None
    depth_data: Optional[Dict[str, Any]] = None
    
    # 압축된 이미지 (Base64 또는 바이트)
    image_base64: Optional[str] = None
    image_bytes: Optional[bytes] = None


class IntegratedRGBDCameraSensor:
    """통합 RGB-D 카메라 센서 시스템"""
    
    def __init__(self,
                 camera_path: str = "/World/Sensors/FrontCam_Mount/FrontCam",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0,
                 depth_range: Tuple[float, float] = (0.1, 100.0),
                 synchronization_mode: str = "soft"):  # "soft", "hard", "sequential"
        """
        통합 RGB-D 카메라 센서 초기화
        
        Args:
            camera_path: 카메라 프림 경로
            resolution: 이미지 해상도
            output_dir: 출력 디렉토리
            fps: 목표 프레임레이트
            depth_range: 깊이 범위 (미터)
            synchronization_mode: 동기화 모드
                - "soft": 소프트웨어 동기화 (순차 캡처)
                - "hard": 하드웨어 동기화 (동시 캡처)
                - "sequential": 순차 캡처 (RGB -> Depth)
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/integrated_rgbd_camera"
        self.fps = fps
        self.depth_range = depth_range
        self.synchronization_mode = synchronization_mode
        
        # 개별 센서 인스턴스 (동적 로딩)
        self.rgb_sensor = None
        self.depth_sensor = None
        
        # Isaac Sim 모듈들
        self.rep = None
        self.render_product = None
        
        # 성능 메트릭
        self.capture_count = 0
        self.last_sync_time = 0.0
        self.sync_errors = 0
        self.max_sync_tolerance_ms = 10.0  # 최대 동기화 허용 오차
        
        # 동기화 설정
        self.frame_buffer_size = 3
        self.capture_timeout_ms = 1000.0  # 1초 타임아웃
        
        # 압축 설정 (기본값)
        self.default_rgb_format = "JPEG"
        self.default_rgb_quality = 85
        self.default_depth_format = "PNG16"
        self.default_depth_quality = 1
        
        # 스레드 풀 (비동기 처리용)
        self.executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="RGBD")
        self.capture_lock = threading.Lock()
        
        # 결과 큐
        self.result_queue = queue.Queue(maxsize=10)
        
        logger.info(f"IntegratedRGBDCameraSensor 초기화: {camera_path}")
        logger.info(f"  해상도: {resolution}, 동기화 모드: {synchronization_mode}")
    
    def initialize(self) -> bool:
        """
        통합 RGB-D 센서 초기화
        
        Returns:
            초기화 성공 여부
        """
        try:
            # 출력 디렉토리 생성
            os.makedirs(self.output_dir, exist_ok=True)
            
            # 개별 센서들 동적 로딩 및 초기화
            success = self._initialize_sensors()
            if not success:
                logger.error("개별 센서 초기화 실패")
                return False
            
            # Isaac Sim 공통 모듈 초기화
            import omni.replicator.core as rep
            self.rep = rep
            
            # 통합 카메라 정보 저장
            self._save_integrated_camera_info()
            
            logger.info("통합 RGB-D 센서 초기화 성공")
            return True
            
        except Exception as e:
            logger.error(f"통합 RGB-D 센서 초기화 실패: {e}")
            return False
    
    def _initialize_sensors(self) -> bool:
        """개별 RGB/Depth 센서 초기화"""
        try:
            # 동적 import 방식으로 센서 클래스 로딩
            import importlib.util
            
            # RGB 센서 로딩
            rgb_spec = importlib.util.spec_from_file_location(
                "enhanced_rgb_camera_sensor", 
                "/home/roarm_m3/dev_roarm/roarm_mcp/sensors/enhanced_rgb_camera_sensor.py"
            )
            rgb_module = importlib.util.module_from_spec(rgb_spec)
            rgb_spec.loader.exec_module(rgb_module)
            
            self.rgb_sensor = rgb_module.EnhancedRGBCameraSensor(
                camera_path=self.camera_path,
                resolution=self.resolution,
                output_dir=f"{self.output_dir}/rgb",
                fps=self.fps
            )
            
            # Depth 센서 로딩
            depth_spec = importlib.util.spec_from_file_location(
                "enhanced_depth_camera_sensor", 
                "/home/roarm_m3/dev_roarm/roarm_mcp/sensors/enhanced_depth_camera_sensor.py"
            )
            depth_module = importlib.util.module_from_spec(depth_spec)
            depth_spec.loader.exec_module(depth_module)
            
            self.depth_sensor = depth_module.EnhancedDepthCameraSensor(
                camera_path=self.camera_path,
                resolution=self.resolution,
                output_dir=f"{self.output_dir}/depth",
                fps=self.fps,
                depth_range=self.depth_range
            )
            
            # 개별 센서 초기화
            rgb_init = self.rgb_sensor.initialize()
            depth_init = self.depth_sensor.initialize()
            
            if not rgb_init:
                logger.error("RGB 센서 초기화 실패")
                return False
            
            if not depth_init:
                logger.error("Depth 센서 초기화 실패")
                return False
            
            logger.info("✅ RGB 및 Depth 센서 초기화 성공")
            return True
            
        except Exception as e:
            logger.error(f"개별 센서 초기화 중 오류: {e}")
            return False
    
    def _save_integrated_camera_info(self):
        """통합 카메라 정보 저장"""
        if not self.rgb_sensor or not self.depth_sensor:
            return
        
        integrated_info = {
            "camera_name": "isaac_sim_rgbd_camera",
            "synchronization_mode": self.synchronization_mode,
            "target_fps": self.fps,
            "resolution": {
                "width": self.resolution[0],
                "height": self.resolution[1]
            },
            
            # RGB 카메라 정보
            "rgb_camera": {
                "camera_matrix": self.rgb_sensor.camera_matrix.tolist(),
                "distortion_coeffs": self.rgb_sensor.distortion_coeffs.tolist(),
                "horizontal_aperture_mm": self.rgb_sensor.horizontal_aperture,
                "vertical_aperture_mm": self.rgb_sensor.vertical_aperture,
                "focal_length_mm": self.rgb_sensor.focal_length,
                "default_format": self.default_rgb_format,
                "default_quality": self.default_rgb_quality
            },
            
            # Depth 카메라 정보 (RGB와 정렬됨)
            "depth_camera": {
                "camera_matrix": self.depth_sensor.camera_matrix.tolist(),
                "distortion_coeffs": self.depth_sensor.distortion_coeffs.tolist(),
                "horizontal_aperture_mm": self.depth_sensor.horizontal_aperture,
                "vertical_aperture_mm": self.depth_sensor.vertical_aperture,
                "focal_length_mm": self.depth_sensor.focal_length,
                "min_depth_m": self.depth_range[0],
                "max_depth_m": self.depth_range[1],
                "depth_precision_bits": 16,
                "default_format": self.default_depth_format,
                "default_quality": self.default_depth_quality
            },
            
            # Isaac Sim 특화 정보
            "isaac_sim": {
                "camera_path": self.camera_path,
                "render_product_id": None,  # 런타임에 업데이트
                "synchronization_tolerance_ms": self.max_sync_tolerance_ms
            },
            
            # 성능 설정
            "performance": {
                "frame_buffer_size": self.frame_buffer_size,
                "capture_timeout_ms": self.capture_timeout_ms,
                "thread_pool_workers": self.executor._max_workers
            }
        }
        
        info_path = Path(self.output_dir) / "integrated_camera_info.json"
        with open(info_path, 'w') as f:
            json.dump(integrated_info, f, indent=2)
        
        logger.info(f"통합 카메라 정보 저장: {info_path}")
    
    def _capture_rgb_async(self, 
                          format: str = None, 
                          quality: int = None, 
                          method: str = "direct") -> CameraCaptureResult:
        """비동기 RGB 캡처"""
        start_time = time.time()
        
        try:
            result = self.rgb_sensor.capture_and_encode_image(
                format=format or self.default_rgb_format,
                quality=quality or self.default_rgb_quality,
                base64_encode=True,
                method=method
            )
            
            if result:
                return CameraCaptureResult(
                    timestamp=result["timestamp"],
                    sensor_type="rgb",
                    resolution=tuple(result["resolution"]),
                    format=result["format"],
                    size_bytes=result["size_bytes"],
                    capture_time_ms=result["capture_time_ms"],
                    capture_method=result["capture_method"],
                    success=True,
                    rgb_data=result,
                    image_base64=result.get("image_base64")
                )
            else:
                return CameraCaptureResult(
                    timestamp=time.time(),
                    sensor_type="rgb",
                    resolution=self.resolution,
                    format=format or self.default_rgb_format,
                    size_bytes=0,
                    capture_time_ms=(time.time() - start_time) * 1000,
                    capture_method=method,
                    success=False,
                    error_message="RGB 캡처 실패"
                )
                
        except Exception as e:
            return CameraCaptureResult(
                timestamp=time.time(),
                sensor_type="rgb",
                resolution=self.resolution,
                format=format or self.default_rgb_format,
                size_bytes=0,
                capture_time_ms=(time.time() - start_time) * 1000,
                capture_method=method,
                success=False,
                error_message=f"RGB 캡처 오류: {e}"
            )
    
    def _capture_depth_async(self, 
                           format: str = None, 
                           quality: int = None, 
                           method: str = "direct") -> CameraCaptureResult:
        """비동기 Depth 캡처"""
        start_time = time.time()
        
        try:
            result = self.depth_sensor.capture_and_encode_depth_image(
                format=format or self.default_depth_format,
                quality=quality or self.default_depth_quality,
                base64_encode=True,
                method=method
            )
            
            if result:
                return CameraCaptureResult(
                    timestamp=result["timestamp"],
                    sensor_type="depth",
                    resolution=tuple(result["resolution"]),
                    format=result["format"],
                    size_bytes=result["size_bytes"],
                    capture_time_ms=result["capture_time_ms"],
                    capture_method=result["capture_method"],
                    success=True,
                    depth_data=result,
                    image_base64=result.get("depth_image_base64")
                )
            else:
                return CameraCaptureResult(
                    timestamp=time.time(),
                    sensor_type="depth",
                    resolution=self.resolution,
                    format=format or self.default_depth_format,
                    size_bytes=0,
                    capture_time_ms=(time.time() - start_time) * 1000,
                    capture_method=method,
                    success=False,
                    error_message="Depth 캡처 실패"
                )
                
        except Exception as e:
            return CameraCaptureResult(
                timestamp=time.time(),
                sensor_type="depth",
                resolution=self.resolution,
                format=format or self.default_depth_format,
                size_bytes=0,
                capture_time_ms=(time.time() - start_time) * 1000,
                capture_method=method,
                success=False,
                error_message=f"Depth 캡처 오류: {e}"
            )
    
    def capture_synchronized_rgbd(self,
                                rgb_format: str = None,
                                rgb_quality: int = None,
                                depth_format: str = None,
                                depth_quality: int = None,
                                method: str = "direct") -> Dict[str, Any]:
        """
        동기화된 RGB-D 이미지 캡처
        
        Args:
            rgb_format: RGB 압축 포맷
            rgb_quality: RGB 압축 품질
            depth_format: Depth 압축 포맷  
            depth_quality: Depth 압축 품질
            method: 캡처 방법
            
        Returns:
            동기화된 RGB-D 데이터 딕셔너리
        """
        with self.capture_lock:
            sync_start_time = time.time()
            
            try:
                if self.synchronization_mode == "sequential":
                    # 순차 캡처 (RGB -> Depth)
                    rgb_result = self._capture_rgb_async(rgb_format, rgb_quality, method)
                    depth_result = self._capture_depth_async(depth_format, depth_quality, method)
                    
                    # 타임스탬프 동기화 (평균값 사용)
                    sync_timestamp = (rgb_result.timestamp + depth_result.timestamp) / 2
                    
                elif self.synchronization_mode in ["soft", "hard"]:
                    # 병렬 캡처 (동시)
                    with ThreadPoolExecutor(max_workers=2) as executor:
                        rgb_future = executor.submit(
                            self._capture_rgb_async, rgb_format, rgb_quality, method
                        )
                        depth_future = executor.submit(
                            self._capture_depth_async, depth_format, depth_quality, method
                        )
                        
                        # 결과 대기 (타임아웃 적용)
                        rgb_result = rgb_future.result(timeout=self.capture_timeout_ms/1000)
                        depth_result = depth_future.result(timeout=self.capture_timeout_ms/1000)
                        
                        sync_timestamp = min(rgb_result.timestamp, depth_result.timestamp)
                
                else:
                    raise ValueError(f"지원하지 않는 동기화 모드: {self.synchronization_mode}")
                
                # 동기화 품질 검증
                time_diff_ms = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
                sync_quality = "good" if time_diff_ms <= self.max_sync_tolerance_ms else "poor"
                
                if sync_quality == "poor":
                    self.sync_errors += 1
                    logger.warning(f"⚠️ 동기화 품질 저하: {time_diff_ms:.1f}ms 차이")
                
                # 성공 여부 확인
                overall_success = rgb_result.success and depth_result.success
                
                # 성능 메트릭 업데이트
                self.capture_count += 1
                self.last_sync_time = time.time() - sync_start_time
                
                # 통합 결과 생성
                integrated_result = {
                    "sync_timestamp": sync_timestamp,
                    "sync_mode": self.synchronization_mode,
                    "sync_time_ms": self.last_sync_time * 1000,
                    "sync_quality": sync_quality,
                    "time_difference_ms": time_diff_ms,
                    "overall_success": overall_success,
                    
                    # RGB 데이터
                    "rgb": asdict(rgb_result),
                    
                    # Depth 데이터  
                    "depth": asdict(depth_result),
                    
                    # 메타데이터
                    "metadata": {
                        "capture_count": self.capture_count,
                        "sync_error_count": self.sync_errors,
                        "sync_error_rate": self.sync_errors / self.capture_count if self.capture_count > 0 else 0,
                        "camera_path": self.camera_path,
                        "resolution": list(self.resolution)
                    }
                }
                
                logger.info(f"RGB-D 동기화 캡처 완료: {sync_quality} 품질, "
                           f"{self.last_sync_time*1000:.1f}ms, 차이: {time_diff_ms:.1f}ms")
                
                return integrated_result
                
            except Exception as e:
                logger.error(f"동기화된 RGB-D 캡처 실패: {e}")
                return {
                    "sync_timestamp": time.time(),
                    "sync_mode": self.synchronization_mode,
                    "sync_time_ms": (time.time() - sync_start_time) * 1000,
                    "sync_quality": "failed",
                    "overall_success": False,
                    "error_message": str(e),
                    "rgb": None,
                    "depth": None
                }
    
    def capture_rgbd_stream(self,
                          duration_seconds: float = 10.0,
                          target_fps: float = None,
                          **kwargs) -> List[Dict[str, Any]]:
        """
        연속 RGB-D 스트림 캡처
        
        Args:
            duration_seconds: 캡처 지속 시간
            target_fps: 목표 FPS (None이면 기본값 사용)
            **kwargs: capture_synchronized_rgbd에 전달할 추가 인수
            
        Returns:
            캡처된 프레임들의 리스트
        """
        fps = target_fps or self.fps
        frame_interval = 1.0 / fps
        
        frames = []
        start_time = time.time()
        frame_count = 0
        
        logger.info(f"RGB-D 스트림 캡처 시작: {duration_seconds}초, {fps} FPS 목표")
        
        try:
            while (time.time() - start_time) < duration_seconds:
                frame_start = time.time()
                
                # 동기화된 프레임 캡처
                frame_data = self.capture_synchronized_rgbd(**kwargs)
                
                if frame_data["overall_success"]:
                    frames.append(frame_data)
                    frame_count += 1
                    
                    if frame_count % 10 == 0:
                        elapsed = time.time() - start_time
                        actual_fps = frame_count / elapsed
                        logger.info(f"진행상황: {frame_count}프레임, "
                                   f"{elapsed:.1f}초, 실제 FPS: {actual_fps:.1f}")
                
                # FPS 조절을 위한 대기
                frame_time = time.time() - frame_start
                sleep_time = max(0, frame_interval - frame_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
            
            total_time = time.time() - start_time
            actual_fps = len(frames) / total_time
            
            logger.info(f"RGB-D 스트림 캡처 완료: {len(frames)}프레임, "
                       f"실제 FPS: {actual_fps:.1f}")
            
            return frames
            
        except Exception as e:
            logger.error(f"RGB-D 스트림 캡처 중 오류: {e}")
            return frames
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """통합 센서 성능 메트릭"""
        rgb_metrics = self.rgb_sensor.get_performance_metrics() if self.rgb_sensor else {}
        depth_metrics = self.depth_sensor.get_performance_metrics() if self.depth_sensor else {}
        
        return {
            "integrated_metrics": {
                "total_sync_captures": self.capture_count,
                "last_sync_time_ms": self.last_sync_time * 1000,
                "sync_error_count": self.sync_errors,
                "sync_error_rate": self.sync_errors / self.capture_count if self.capture_count > 0 else 0,
                "synchronization_mode": self.synchronization_mode,
                "max_sync_tolerance_ms": self.max_sync_tolerance_ms,
                "target_fps": self.fps,
                "actual_fps": 1.0 / self.last_sync_time if self.last_sync_time > 0 else 0
            },
            "rgb_sensor_metrics": rgb_metrics,
            "depth_sensor_metrics": depth_metrics,
            "camera_info": {
                "camera_path": self.camera_path,
                "resolution": list(self.resolution),
                "depth_range_m": list(self.depth_range),
                "output_directory": self.output_dir
            }
        }
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.rgb_sensor:
                self.rgb_sensor.cleanup()
            if self.depth_sensor:
                self.depth_sensor.cleanup()
            
            # 스레드 풀 정리
            self.executor.shutdown(wait=True)
            
            logger.info("통합 RGB-D 센서 리소스 정리 완료")
            
        except Exception as e:
            logger.error(f"리소스 정리 중 오류: {e}")


def test_integrated_rgbd_system():
    """통합 RGB-D 시스템 테스트"""
    print("🎭 통합 RGB-D 카메라 시스템 테스트 시작")
    
    try:
        # Isaac Sim 초기화 (이전과 동일)
        from isaacsim import SimulationApp
        
        simulation_app = SimulationApp({
            "headless": True,
            "width": 1280,
            "height": 720,
            "renderer": "RayTracedLighting",
            "anti_aliasing": 3,
            "samples_per_pixel": 64,
            "denoiser": True
        })
        
        print("✅ Isaac Sim 초기화 완료")
        
        # USD 환경 설정 (RGB+Depth 테스트에 최적화)
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            stage = Usd.Stage.CreateNew("integrated_test_stage.usd")
            usd_context.attach_stage(stage)
        
        stage.SetMetadata("metersPerUnit", 1.0)
        
        # 기본 환경
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        # 조명
        lights_mount = stage.DefinePrim(Sdf.Path("/World/Lights"), "Xform")
        distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/DistantLight"))
        distant_light.CreateIntensityAttr(8000.0)
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        
        dome_light = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/DomeLight"))
        dome_light.CreateIntensityAttr(2500.0)
        dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.95, 1.0))
        
        # RGB+Depth 모두에 좋은 테스트 객체들
        test_objects = [
            # (이름, 타입, 위치, 크기, 색상)
            ("NearRedCube", "cube", Gf.Vec3f(-2, 0, 3), 0.8, Gf.Vec3f(1, 0, 0)),
            ("MidGreenSphere", "sphere", Gf.Vec3f(0, 0, 8), 1.0, Gf.Vec3f(0, 1, 0)),
            ("FarBlueCylinder", "cylinder", Gf.Vec3f(3, -2, 15), 1.2, Gf.Vec3f(0, 0, 1)),
            ("VeryFarYellowCube", "cube", Gf.Vec3f(-5, 3, 25), 1.5, Gf.Vec3f(1, 1, 0))
        ]
        
        for name, obj_type, pos, size, color in test_objects:
            if obj_type == "sphere":
                obj = UsdGeom.Sphere.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(size)
            elif obj_type == "cylinder":
                obj = UsdGeom.Cylinder.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(size * 0.6)
                obj.CreateHeightAttr(size * 1.5)
            else:  # cube
                obj = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateSizeAttr(size)
            
            obj.AddTranslateOp().Set(pos)
            obj.CreateDisplayColorAttr([color])
        
        # 바닥
        ground = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Ground"))
        ground.CreateSizeAttr(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3f(0, 0, -1))
        ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 1))
        ground.CreateDisplayColorAttr([Gf.Vec3f(0.4, 0.4, 0.4)])
        
        # 카메라 마운트
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(-5.0, 0.0, 2.0))
        cam_mount_xform.SetRotate(Gf.Vec3f(0, 5, 0))
        
        # 카메라 (RGB와 Depth 공통)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        camera_prim.CreateHorizontalApertureAttr(36.0)
        camera_prim.CreateVerticalApertureAttr(20.25)
        camera_prim.CreateFocalLengthAttr(24.0)
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
        
        print("  - RGB+Depth 통합 테스트 환경 생성 완료")
        
        # 시뮬레이션 준비
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        print("🔄 통합 시뮬레이션 환경 준비 중...")
        for i in range(30):
            app.update()
            if i % 10 == 0:
                print(f"  - 프레임 {i+1}/30 업데이트")
        
        print("✅ 통합 시뮬레이션 환경 준비 완료")
        
        # 통합 RGB-D 센서 생성 및 초기화
        rgbd_sensor = IntegratedRGBDCameraSensor(
            camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
            resolution=(1280, 720),
            output_dir="/tmp/integrated_rgbd_test",
            fps=30.0,
            depth_range=(0.1, 100.0),
            synchronization_mode="soft"  # 소프트웨어 동기화
        )
        
        if not rgbd_sensor.initialize():
            print("❌ 통합 RGB-D 센서 초기화 실패")
            return False
        
        print("✅ 통합 RGB-D 센서 초기화 완료")
        
        # 동기화된 캡처 테스트
        print("\n📷📐 동기화된 RGB-D 캡처 테스트...")
        
        sync_test_cases = [
            {"rgb_format": "JPEG", "rgb_quality": 85, "depth_format": "PNG16", "depth_quality": 1},
            {"rgb_format": "JPEG", "rgb_quality": 95, "depth_format": "PNG8", "depth_quality": 3},
            {"rgb_format": "PNG", "rgb_quality": 6, "depth_format": "JPEG8", "depth_quality": 85},
        ]
        
        sync_results = []
        for i, test_case in enumerate(sync_test_cases):
            print(f"\n--- 동기화 테스트 {i+1}/{len(sync_test_cases)} ---")
            print(f"RGB: {test_case['rgb_format']} Q{test_case['rgb_quality']}")
            print(f"Depth: {test_case['depth_format']} Q{test_case['depth_quality']}")
            
            result = rgbd_sensor.capture_synchronized_rgbd(**test_case)
            
            if result["overall_success"]:
                sync_results.append(result)
                
                print(f"  ✅ 동기화 성공: {result['sync_quality']} 품질")
                print(f"  ⏱️  동기화 시간: {result['sync_time_ms']:.1f}ms")
                print(f"  📏 시간 차이: {result['time_difference_ms']:.1f}ms")
                
                # RGB 결과
                rgb = result["rgb"]
                print(f"  📷 RGB: {rgb['format']}, {rgb['size_bytes']} bytes, {rgb['capture_time_ms']:.1f}ms")
                
                # Depth 결과
                depth = result["depth"]
                print(f"  📐 Depth: {depth['format']}, {depth['size_bytes']} bytes, {depth['capture_time_ms']:.1f}ms")
                
            else:
                print(f"  ❌ 동기화 실패: {result.get('error_message', '알 수 없는 오류')}")
            
            time.sleep(0.2)
        
        # 스트림 캡처 테스트 (짧은 시간)
        print(f"\n🎬 RGB-D 스트림 캡처 테스트 (3초)...")
        stream_frames = rgbd_sensor.capture_rgbd_stream(
            duration_seconds=3.0,
            target_fps=10.0,  # 테스트용으로 낮은 FPS
            rgb_format="JPEG",
            rgb_quality=75,
            depth_format="PNG16",
            depth_quality=1
        )
        
        print(f"  ✅ 스트림 캡처 완료: {len(stream_frames)}프레임")
        
        if stream_frames:
            # 스트림 품질 분석
            sync_qualities = [f['sync_quality'] for f in stream_frames if f['overall_success']]
            good_sync = sync_qualities.count('good')
            poor_sync = sync_qualities.count('poor')
            
            total_rgb_size = sum(f['rgb']['size_bytes'] for f in stream_frames if f['overall_success'])
            total_depth_size = sum(f['depth']['size_bytes'] for f in stream_frames if f['overall_success'])
            
            print(f"  📊 스트림 품질: 좋음 {good_sync}, 나쁨 {poor_sync}")
            print(f"  📦 총 데이터: RGB {total_rgb_size:,}B, Depth {total_depth_size:,}B")
        
        # 성능 메트릭 출력
        metrics = rgbd_sensor.get_performance_metrics()
        print(f"\n📊 통합 센서 성능 메트릭:")
        
        integrated = metrics["integrated_metrics"]
        print(f"  동기화 캡처 횟수: {integrated['total_sync_captures']}")
        print(f"  마지막 동기화 시간: {integrated['last_sync_time_ms']:.1f}ms")
        print(f"  동기화 오류율: {integrated['sync_error_rate']:.1%}")
        print(f"  실제 FPS: {integrated['actual_fps']:.1f} (목표: {integrated['target_fps']})")
        
        # 파일 출력 확인
        output_path = Path(rgbd_sensor.output_dir)
        if output_path.exists():
            all_files = list(output_path.rglob("*"))
            rgb_files = list((output_path / "rgb").glob("*")) if (output_path / "rgb").exists() else []
            depth_files = list((output_path / "depth").glob("*")) if (output_path / "depth").exists() else []
            
            print(f"\n📁 생성된 파일들:")
            print(f"  전체: {len(all_files)}개")
            print(f"  RGB: {len(rgb_files)}개")
            print(f"  Depth: {len(depth_files)}개")
            
            # integrated_camera_info.json 확인
            info_file = output_path / "integrated_camera_info.json"
            if info_file.exists():
                print(f"  ✅ 통합 카메라 정보: {info_file.stat().st_size} bytes")
        
        # 리소스 정리
        rgbd_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ 통합 RGB-D 시스템 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ 통합 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_integrated_rgbd_system()
    sys.exit(0 if success else 1)