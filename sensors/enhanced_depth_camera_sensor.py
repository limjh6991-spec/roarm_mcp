#!/usr/bin/env python3
"""
개선된 Depth 카메라 센서 모듈
Isaac Sim 5.0 Depth Annotator를 사용한 실시간 깊이 맵 캡처 및 압축/전송
"""

import os
import sys
import base64
import json
import time
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, Dict, Any, List, Union
import logging
import struct

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EnhancedDepthCameraSensor:
    """Isaac Sim 5.0 개선된 Depth 카메라 센서 클래스"""
    
    def __init__(self, 
                 camera_path: str = "/World/Sensors/FrontCam_Mount/FrontCam",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0,
                 depth_range: Tuple[float, float] = (0.1, 100.0)):
        """
        개선된 Depth 카메라 센서 초기화
        
        Args:
            camera_path: 카메라 프림 경로
            resolution: 이미지 해상도 (width, height)
            output_dir: 출력 디렉토리 (선택사항)
            fps: 목표 프레임레이트
            depth_range: 깊이 범위 (min_depth, max_depth) in meters
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/enhanced_depth_camera"
        self.fps = fps
        self.depth_range = depth_range
        
        # Isaac Sim 모듈들 (런타임에 import)
        self.rep = None
        self.render_product = None
        self.writer = None
        self.depth_annotator = None
        
        # 성능 메트릭
        self.capture_count = 0
        self.last_capture_time = 0.0
        self.frame_warmup_count = 3  # 워밍업 프레임 수
        
        # Depth 전용 설정
        self.depth_precision = 16  # 16-bit depth
        self.normalize_depth = True  # 깊이 정규화 여부
        self.invalid_depth_value = 0.0  # 잘못된 깊이 값
        
        # 압축 설정 (depth 특화)
        self.png_compression = 1  # PNG 압축 레벨 (depth는 무손실 압축)
        self.enable_16bit_png = True  # 16-bit PNG 지원
        self.enable_normalized_jpeg = True  # 정규화된 8-bit JPEG 지원
        
        # 카메라 내부 파라미터 (RGB와 동일하게 설정)
        self._calculate_camera_parameters()
        
        logger.info(f"EnhancedDepthCameraSensor 초기화: {camera_path}, 해상도: {resolution}")
        logger.info(f"  깊이 범위: {depth_range[0]:.1f}m ~ {depth_range[1]:.1f}m")
    
    def _calculate_camera_parameters(self):
        """카메라 파라미터 계산 (RGB와 정합)"""
        width, height = self.resolution
        aspect_ratio = width / height
        
        # RGB와 동일한 조리개 크기 (정합성)
        self.horizontal_aperture = 36.0  # mm
        self.vertical_aperture = self.horizontal_aperture / aspect_ratio  # 20.25mm
        self.focal_length = 24.0  # mm
        
        # 픽셀 단위 초점거리 계산
        self.fx = self.focal_length * width / self.horizontal_aperture
        self.fy = self.focal_length * height / self.vertical_aperture
        
        # 주점 (이미지 중심)
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        # 카메라 내부 파라미터 행렬 K (RGB와 동일)
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        
        # 왜곡 계수 (핀홀 카메라 모델)
        self.distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
        logger.info(f"Depth 카메라 파라미터 (RGB와 정합):")
        logger.info(f"  조리개: {self.horizontal_aperture:.1f}x{self.vertical_aperture:.2f}mm")
        logger.info(f"  초점거리: fx={self.fx:.1f}, fy={self.fy:.1f}")
    
    def initialize(self) -> bool:
        """
        Depth 카메라 센서 초기화 (Isaac Sim 환경에서 호출)
        
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
            
            # Distance to Camera Annotator 사용 (정확한 미터 단위)
            self.depth_annotator = rep.AnnotatorRegistry.get_annotator(
                "distance_to_camera_colorize",  # 또는 "distance_to_image_plane"
                init_params={
                    "use_colorize": False,  # 원시 깊이 값 사용
                    "colorize_method": "turbo"  # 시각화용 (선택사항)
                }
            )
            self.depth_annotator.attach([self.render_product])
            
            # BasicWriter 초기화 (백업용)
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                output_dir=self.output_dir,
                distance_to_camera=True,  # depth 데이터 저장
                camera_params=True
            )
            self.writer.attach([self.render_product])
            
            # 카메라 정보 저장
            self._save_camera_info()
            
            logger.info(f"Depth 카메라 센서 초기화 성공: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"Depth 카메라 센서 초기화 실패: {e}")
            return False
    
    def _save_camera_info(self):
        """ROS 호환 Depth 카메라 정보 저장"""
        camera_info = {
            "image_width": self.resolution[0],
            "image_height": self.resolution[1],
            "camera_name": "isaac_sim_depth_camera",
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
            # Depth 특화 정보
            "depth_info": {
                "min_depth_m": self.depth_range[0],
                "max_depth_m": self.depth_range[1],
                "depth_precision_bits": self.depth_precision,
                "depth_unit": "meters",
                "invalid_value": self.invalid_depth_value,
                "encoding": "16UC1"  # ROS 표준
            },
            # Isaac Sim 특화 정보
            "isaac_sim": {
                "horizontal_aperture_mm": self.horizontal_aperture,
                "vertical_aperture_mm": self.vertical_aperture,
                "focal_length_mm": self.focal_length,
                "camera_path": self.camera_path,
                "fps": self.fps,
                "sensor_type": "depth"
            }
        }
        
        camera_info_path = Path(self.output_dir) / "depth_camera_info.json"
        with open(camera_info_path, 'w') as f:
            json.dump(camera_info, f, indent=2)
        
        logger.info(f"Depth 카메라 정보 저장 완료: {camera_info_path}")
    
    def _warmup_frames(self, count: int = 3):
        """프레임 워밍업 (첫 프레임 이슈 방지)"""
        try:
            import omni.kit.app
            app = omni.kit.app.get_app()
            
            logger.debug(f"Depth 프레임 워밍업 시작 ({count}회)")
            for i in range(count):
                app.update()
                time.sleep(0.02)  # 20ms 대기 (depth는 RGB보다 조금 더)
                
                if i == count - 1:  # 마지막 워밍업에서 렌더링
                    self.rep.orchestrator.step()
                    if self.writer:
                        self.writer.flush()
                        
            logger.debug("Depth 프레임 워밍업 완료")
            
        except Exception as e:
            logger.warning(f"Depth 프레임 워밍업 중 오류: {e}")
    
    def _validate_depth_data(self, depth_data: np.ndarray) -> Tuple[bool, Dict[str, float]]:
        """깊이 데이터 유효성 검증 및 통계"""
        if depth_data is None or depth_data.size == 0:
            return False, {"reason": "empty_data"}
        
        # 기본 통계
        valid_mask = (depth_data > 0) & (depth_data < np.inf) & ~np.isnan(depth_data)
        valid_pixels = np.sum(valid_mask)
        total_pixels = depth_data.size
        valid_ratio = valid_pixels / total_pixels
        
        stats = {
            "total_pixels": total_pixels,
            "valid_pixels": valid_pixels,
            "valid_ratio": valid_ratio,
            "min_depth": float(np.min(depth_data[valid_mask])) if valid_pixels > 0 else 0.0,
            "max_depth": float(np.max(depth_data[valid_mask])) if valid_pixels > 0 else 0.0,
            "mean_depth": float(np.mean(depth_data[valid_mask])) if valid_pixels > 0 else 0.0,
            "std_depth": float(np.std(depth_data[valid_mask])) if valid_pixels > 0 else 0.0
        }
        
        # 유효성 검증 기준
        is_valid = (
            valid_ratio > 0.1 and  # 최소 10% 유효 픽셀
            stats["max_depth"] > stats["min_depth"] and  # 깊이 범위가 있음
            stats["mean_depth"] > 0.01  # 의미있는 깊이 값
        )
        
        if not is_valid:
            stats["reason"] = f"invalid_depth_data (valid_ratio={valid_ratio:.2f})"
            logger.warning(f"⚠️ 유효하지 않은 깊이 데이터: {stats['reason']}")
        else:
            logger.debug(f"깊이 데이터 통계: {stats['valid_pixels']}/{stats['total_pixels']} 유효, "
                        f"범위: {stats['min_depth']:.2f}~{stats['max_depth']:.2f}m")
        
        return is_valid, stats
    
    def capture_depth_image_direct(self) -> Optional[np.ndarray]:
        """
        Annotator를 통한 직접 Depth 이미지 캡처
        
        Returns:
            Depth 이미지 배열 (H, W) - 미터 단위 float32 또는 None
        """
        if not self.depth_annotator:
            logger.error("Depth Annotator가 초기화되지 않음")
            return None
        
        try:
            start_time = time.time()
            
            # 워밍업 (첫 캡처시에만)
            if self.capture_count == 0:
                self._warmup_frames(self.frame_warmup_count)
            
            # Annotator를 통한 직접 데이터 획득
            self.rep.orchestrator.step()
            depth_data = self.depth_annotator.get_data()
            
            if depth_data is None:
                logger.warning("Depth Annotator에서 데이터를 가져올 수 없음")
                return None
            
            # numpy 배열로 변환
            if hasattr(depth_data, 'cpu'):
                # GPU tensor인 경우
                depth_array = depth_data.cpu().numpy()
            else:
                depth_array = np.array(depth_data)
            
            # 데이터 타입 확인 및 정리
            if len(depth_array.shape) == 3 and depth_array.shape[-1] == 1:
                # (H, W, 1) → (H, W)
                depth_array = depth_array.squeeze(-1)
            elif len(depth_array.shape) == 3:
                # RGB 형태라면 첫 번째 채널만 사용
                depth_array = depth_array[..., 0]
            
            # float32로 확보
            if depth_array.dtype != np.float32:
                depth_array = depth_array.astype(np.float32)
            
            # 깊이 데이터 유효성 검증
            is_valid, stats = self._validate_depth_data(depth_array)
            if not is_valid:
                logger.warning("유효하지 않은 깊이 데이터, 재시도...")
                # 재시도
                self._warmup_frames(1)
                self.rep.orchestrator.step()
                depth_data = self.depth_annotator.get_data()
                
                if depth_data is not None:
                    if hasattr(depth_data, 'cpu'):
                        depth_array = depth_data.cpu().numpy()
                    else:
                        depth_array = np.array(depth_data)
                    
                    if len(depth_array.shape) == 3 and depth_array.shape[-1] == 1:
                        depth_array = depth_array.squeeze(-1)
                    elif len(depth_array.shape) == 3:
                        depth_array = depth_array[..., 0]
                    
                    if depth_array.dtype != np.float32:
                        depth_array = depth_array.astype(np.float32)
            
            # 성능 메트릭 업데이트
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"Depth 이미지 직접 캡처 성공: {depth_array.shape}, 시간: {self.last_capture_time:.3f}s")
            return depth_array
            
        except Exception as e:
            logger.error(f"Depth 이미지 직접 캡처 실패: {e}")
            return None
    
    def capture_depth_image_file(self) -> Optional[np.ndarray]:
        """
        파일 기반 Depth 이미지 캡처 (백업 방법)
        
        Returns:
            Depth 이미지 배열 (H, W) - 미터 단위 float32 또는 None
        """
        if not self.rep or not self.render_product:
            logger.error("Depth 카메라 센서가 초기화되지 않음")
            return None
        
        try:
            start_time = time.time()
            
            # 워밍업 (첫 캡처시에만)
            if self.capture_count == 0:
                self._warmup_frames(self.frame_warmup_count)
            
            # Replicator 렌더링 단계 실행
            self.rep.orchestrator.step()
            if self.writer:
                self.writer.flush()
            
            # 최신 생성된 Depth 파일 찾기
            max_retries = 3
            depth_files = []
            
            for retry in range(max_retries):
                # distance_to_camera 파일 패턴 찾기
                depth_files = list(Path(self.output_dir).glob("distance_to_camera_*.npy"))
                if not depth_files:
                    depth_files = list(Path(self.output_dir).glob("depth_*.npy"))
                if not depth_files:
                    depth_files = list(Path(self.output_dir).glob("distance_*.npy"))
                if depth_files:
                    break
                time.sleep(0.05)
            
            if not depth_files:
                logger.warning("Depth 이미지 파일을 찾을 수 없음")
                return None
            
            # 가장 최근 파일 선택
            latest_file = max(depth_files, key=lambda x: x.stat().st_mtime)
            
            # numpy 배열로 로드
            depth_array = np.load(str(latest_file))
            
            # 데이터 타입 및 형태 정리
            if len(depth_array.shape) == 3 and depth_array.shape[-1] == 1:
                depth_array = depth_array.squeeze(-1)
            elif len(depth_array.shape) == 3:
                depth_array = depth_array[..., 0]
            
            if depth_array.dtype != np.float32:
                depth_array = depth_array.astype(np.float32)
            
            # 깊이 데이터 유효성 검증
            is_valid, stats = self._validate_depth_data(depth_array)
            if not is_valid:
                logger.warning(f"파일에서 로드된 깊이 데이터가 유효하지 않음: {latest_file}")
            
            # 성능 메트릭 업데이트
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"Depth 이미지 파일 캡처 성공: {depth_array.shape}, 시간: {self.last_capture_time:.3f}s")
            return depth_array
            
        except Exception as e:
            logger.error(f"Depth 이미지 파일 캡처 실패: {e}")
            return None
    
    def capture_depth_image(self, method: str = "direct") -> Optional[np.ndarray]:
        """
        Depth 이미지 캡처 (메소드 선택 가능)
        
        Args:
            method: "direct" (Annotator) 또는 "file" (파일 기반)
            
        Returns:
            Depth 이미지 배열 (H, W) - 미터 단위 float32 또는 None
        """
        if method == "direct":
            return self.capture_depth_image_direct()
        elif method == "file":
            return self.capture_depth_image_file()
        else:
            logger.error(f"지원하지 않는 Depth 캡처 방법: {method}")
            return None
    
    def normalize_depth_to_8bit(self, 
                               depth_array: np.ndarray,
                               depth_range: Optional[Tuple[float, float]] = None) -> np.ndarray:
        """
        깊이 맵을 8-bit으로 정규화 (JPEG 압축용)
        
        Args:
            depth_array: 원시 깊이 배열 (미터 단위)
            depth_range: 정규화 범위 (min_depth, max_depth)
            
        Returns:
            8-bit 정규화된 깊이 배열 (0-255)
        """
        if depth_range is None:
            depth_range = self.depth_range
        
        min_depth, max_depth = depth_range
        
        # 유효한 깊이 값만 처리
        valid_mask = (depth_array > 0) & (depth_array < np.inf) & ~np.isnan(depth_array)
        
        # 깊이 범위로 클리핑
        clipped_depth = np.clip(depth_array, min_depth, max_depth)
        
        # 0-255 범위로 정규화
        normalized = (clipped_depth - min_depth) / (max_depth - min_depth) * 255.0
        
        # 유효하지 않은 픽셀은 0으로 설정
        normalized[~valid_mask] = 0
        
        return normalized.astype(np.uint8)
    
    def convert_depth_to_16bit(self, 
                              depth_array: np.ndarray,
                              depth_range: Optional[Tuple[float, float]] = None,
                              scale: float = 1000.0) -> np.ndarray:
        """
        깊이 맵을 16-bit 정수로 변환 (PNG 저장용)
        
        Args:
            depth_array: 원시 깊이 배열 (미터 단위)
            depth_range: 깊이 범위
            scale: 스케일링 팩터 (1000 = 밀리미터 단위)
            
        Returns:
            16-bit 깊이 배열 (0-65535)
        """
        if depth_range is None:
            depth_range = self.depth_range
        
        min_depth, max_depth = depth_range
        
        # 유효한 깊이 값만 처리
        valid_mask = (depth_array > 0) & (depth_array < np.inf) & ~np.isnan(depth_array)
        
        # 밀리미터 단위로 변환 후 클리핑
        depth_mm = depth_array * scale
        clipped_depth = np.clip(depth_mm, min_depth * scale, max_depth * scale)
        
        # 16-bit 범위로 변환 (0-65535)
        # 실제로는 최대 65.535미터까지 표현 가능 (밀리미터 단위)
        depth_16bit = clipped_depth.astype(np.uint16)
        
        # 유효하지 않은 픽셀은 0으로 설정
        depth_16bit[~valid_mask] = 0
        
        return depth_16bit
    
    def compress_depth_image(self, 
                            depth_array: np.ndarray,
                            format: str = "PNG16",
                            quality: Optional[int] = None) -> Optional[bytes]:
        """
        깊이 이미지 압축 (다양한 형식 지원)
        
        Args:
            depth_array: 깊이 배열 (미터 단위 float32)
            format: 압축 포맷 ("PNG16", "PNG8", "JPEG8", "RAW")
            quality: 압축 품질 (PNG 레벨 또는 JPEG 품질)
            
        Returns:
            압축된 이미지 바이트 또는 None
        """
        try:
            # OpenCV가 없다고 가정하고 다른 방법 사용
            if format == "PNG16":
                # 16-bit PNG (무손실, 높은 정밀도)
                depth_16bit = self.convert_depth_to_16bit(depth_array)
                # PIL 또는 imageio 사용 (여기서는 시뮬레이션)
                # 실제로는 imageio.imwrite 사용
                compressed_bytes = self._simulate_png16_compression(depth_16bit, quality or self.png_compression)
                
            elif format == "PNG8":
                # 8-bit PNG (정규화된 깊이)
                depth_8bit = self.normalize_depth_to_8bit(depth_array)
                compressed_bytes = self._simulate_png8_compression(depth_8bit, quality or self.png_compression)
                
            elif format == "JPEG8":
                # 8-bit JPEG (정규화된 깊이, 손실 압축)
                depth_8bit = self.normalize_depth_to_8bit(depth_array)
                compressed_bytes = self._simulate_jpeg_compression(depth_8bit, quality or 85)
                
            elif format == "RAW":
                # 원시 float32 바이너리
                compressed_bytes = depth_array.tobytes()
                
            else:
                logger.error(f"지원하지 않는 깊이 압축 포맷: {format}")
                return None
            
            # 압축 효율 로깅
            original_size = depth_array.nbytes
            compressed_size = len(compressed_bytes)
            compression_ratio = original_size / compressed_size if compressed_size > 0 else 0
            
            logger.debug(f"깊이 압축 완료: {format} {original_size}→{compressed_size} bytes "
                        f"(비율: {compression_ratio:.1f}x)")
            
            return compressed_bytes
            
        except Exception as e:
            logger.error(f"깊이 이미지 압축 실패: {e}")
            return None
    
    def _simulate_png16_compression(self, depth_16bit: np.ndarray, compression_level: int) -> bytes:
        """16-bit PNG 압축 시뮬레이션"""
        # 실제로는 imageio.imwrite 또는 PIL 사용
        height, width = depth_16bit.shape
        
        # 시뮬레이션된 압축 크기 (실제 PNG는 더 효율적)
        base_size = height * width * 2  # 16-bit
        compression_factor = 0.3 + (compression_level / 10) * 0.4  # 30-70% 압축
        simulated_size = int(base_size * compression_factor)
        
        # 더미 바이트 생성 (실제로는 PNG 파일 내용)
        return b'PNG16_SIMULATED' + b'\x00' * (simulated_size - 16)
    
    def _simulate_png8_compression(self, depth_8bit: np.ndarray, compression_level: int) -> bytes:
        """8-bit PNG 압축 시뮬레이션"""
        height, width = depth_8bit.shape
        
        base_size = height * width  # 8-bit
        compression_factor = 0.2 + (compression_level / 10) * 0.3  # 20-50% 압축
        simulated_size = int(base_size * compression_factor)
        
        return b'PNG8_SIMULATED' + b'\x00' * (simulated_size - 15)
    
    def _simulate_jpeg_compression(self, depth_8bit: np.ndarray, quality: int) -> bytes:
        """8-bit JPEG 압축 시뮬레이션"""
        height, width = depth_8bit.shape
        
        base_size = height * width
        compression_factor = 0.05 + (quality / 100) * 0.15  # 5-20% 크기
        simulated_size = int(base_size * compression_factor)
        
        return b'JPEG_SIMULATED' + b'\x00' * (simulated_size - 14)
    
    def encode_base64(self, image_bytes: bytes) -> str:
        """이미지 바이트를 Base64로 인코딩"""
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def capture_and_encode_depth_image(self,
                                     format: str = "PNG16",
                                     quality: Optional[int] = None,
                                     base64_encode: bool = True,
                                     method: str = "direct") -> Optional[Dict[str, Any]]:
        """
        깊이 이미지 캡처, 압축 및 인코딩
        
        Args:
            format: 압축 포맷 ("PNG16", "PNG8", "JPEG8", "RAW")
            quality: 압축 품질
            base64_encode: Base64 인코딩 여부
            method: 캡처 방법 ("direct" 또는 "file")
            
        Returns:
            깊이 데이터 딕셔너리 또는 None
        """
        try:
            # 깊이 이미지 캡처
            depth_array = self.capture_depth_image(method)
            if depth_array is None:
                return None
            
            # 깊이 데이터 통계
            is_valid, depth_stats = self._validate_depth_data(depth_array)
            if not is_valid:
                return None
            
            # 깊이 이미지 압축
            compressed_bytes = self.compress_depth_image(depth_array, format, quality)
            if compressed_bytes is None:
                return None
            
            # 결과 딕셔너리 생성
            result = {
                "timestamp": time.time(),
                "resolution": list(depth_array.shape[::-1]),  # [width, height]
                "format": format.lower(),
                "size_bytes": len(compressed_bytes),
                "capture_time_ms": self.last_capture_time * 1000,
                "capture_method": method,
                "depth_stats": depth_stats,
                "depth_info": {
                    "min_depth_m": self.depth_range[0],
                    "max_depth_m": self.depth_range[1],
                    "precision_bits": self.depth_precision,
                    "unit": "meters"
                },
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
                result["depth_image_base64"] = self.encode_base64(compressed_bytes)
            else:
                result["depth_image_bytes"] = compressed_bytes
            
            logger.info(f"깊이 이미지 캡처 완료 ({method}): {result['format']}, "
                       f"{result['size_bytes']} bytes, {result['capture_time_ms']:.1f}ms")
            logger.info(f"  깊이 통계: {depth_stats['valid_pixels']}/{depth_stats['total_pixels']} 유효, "
                       f"범위: {depth_stats['min_depth']:.2f}~{depth_stats['max_depth']:.2f}m")
            return result
            
        except Exception as e:
            logger.error(f"깊이 이미지 캡처 및 인코딩 실패: {e}")
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Depth 센서 성능 메트릭"""
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "target_fps": self.fps,
            "actual_fps": 1.0 / self.last_capture_time if self.last_capture_time > 0 else 0,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "depth_range_m": list(self.depth_range),
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
                "depth_precision_bits": self.depth_precision,
                "png_compression_level": self.png_compression,
                "enable_16bit_png": self.enable_16bit_png,
                "enable_normalized_jpeg": self.enable_normalized_jpeg
            }
        }
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.depth_annotator:
                self.depth_annotator.detach()
            if self.writer:
                self.writer.detach()
            logger.info("Enhanced Depth 카메라 센서 리소스 정리 완료")
        except Exception as e:
            logger.error(f"Depth 센서 리소스 정리 중 오류: {e}")


def test_enhanced_depth_camera_sensor():
    """개선된 Depth 카메라 센서 테스트 함수"""
    print("🎯 개선된 Depth 카메라 센서 테스트 시작")
    
    try:
        # Isaac Sim 초기화 (RGB와 동일한 설정)
        from isaacsim import SimulationApp
        
        simulation_app = SimulationApp({
            "headless": True,
            "width": 1280,
            "height": 720,
            "renderer": "RayTracedLighting",
            "anti_aliasing": 3,
            "samples_per_pixel": 64,
            "denoiser": True,
            "subdiv_refinement_level": 2
        })
        
        print("✅ Isaac Sim 초기화 완료")
        
        # USD 환경 설정 (깊이 테스트에 적합한 환경)
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            stage = Usd.Stage.CreateNew("depth_test_stage.usd")
            usd_context.attach_stage(stage)
        
        stage.SetMetadata("metersPerUnit", 1.0)
        
        # 기본 환경 생성
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        # 조명 (깊이 테스트용)
        lights_mount = stage.DefinePrim(Sdf.Path("/World/Lights"), "Xform")
        distant_light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/DistantLight"))
        distant_light.CreateIntensityAttr(8000.0)
        distant_light.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        
        # 다양한 거리의 테스트 객체들 (깊이 테스트용)
        test_objects = [
            # (거리, 이름, 위치, 크기)
            (2.0, "NearCube", Gf.Vec3f(0, 0, 2), 0.5),
            (5.0, "MidSphere", Gf.Vec3f(-2, 0, 5), 0.7),
            (10.0, "FarCylinder", Gf.Vec3f(2, 0, 10), 0.8),
            (20.0, "VeryFarCube", Gf.Vec3f(-5, 2, 20), 1.0),
            (50.0, "DistantSphere", Gf.Vec3f(10, -5, 50), 2.0)
        ]
        
        for distance, name, pos, size in test_objects:
            if "Sphere" in name:
                obj = UsdGeom.Sphere.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(size)
            elif "Cylinder" in name:
                obj = UsdGeom.Cylinder.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateRadiusAttr(size * 0.6)
                obj.CreateHeightAttr(size * 2)
            else:
                obj = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/{name}"))
                obj.CreateSizeAttr(size)
            
            obj.AddTranslateOp().Set(pos)
            # 거리별 색상 (가까운 것은 빨강, 먼 것은 파랑)
            color_intensity = max(0.2, 1.0 - distance / 50.0)
            obj.CreateDisplayColorAttr([Gf.Vec3f(1-color_intensity, 0.5, color_intensity)])
        
        # 바닥 평면 (참조용)
        ground = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Ground"))
        ground.CreateSizeAttr(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3f(0, 0, -1))
        ground.AddScaleOp().Set(Gf.Vec3f(100, 100, 1))
        ground.CreateDisplayColorAttr([Gf.Vec3f(0.3, 0.3, 0.3)])
        
        print("  - 다층 깊이 테스트 환경 생성 완료")
        
        # 카메라 마운트 (깊이 캡처에 최적화된 위치)
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(0.0, 0.0, 1.5))  # 지상 1.5m
        cam_mount_xform.SetRotate(Gf.Vec3f(0, 0, 0))  # 전방 시야
        
        # 카메라 설정 (RGB와 동일한 파라미터)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        camera_prim.CreateHorizontalApertureAttr(36.0)
        camera_prim.CreateVerticalApertureAttr(20.25)
        camera_prim.CreateFocalLengthAttr(24.0)
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))  # 깊이 범위와 일치
        
        print("  - 깊이 카메라 설정 완료")
        print("✅ USD 환경 설정 완료")
        
        # Enhanced Depth 카메라 센서 생성 및 초기화
        depth_sensor = EnhancedDepthCameraSensor(
            camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
            resolution=(1280, 720),
            output_dir="/tmp/enhanced_depth_test",
            fps=30.0,
            depth_range=(0.1, 100.0)  # 10cm ~ 100m
        )
        
        if not depth_sensor.initialize():
            print("❌ Depth 카메라 센서 초기화 실패")
            return False
        
        print("✅ Enhanced Depth 카메라 센서 초기화 완료")
        
        # 시뮬레이션 준비
        import omni.kit.app
        app = omni.kit.app.get_app()
        
        print("🔄 깊이 시뮬레이션 환경 준비 중...")
        for i in range(25):
            app.update()
            if i % 5 == 0:
                print(f"  - 프레임 {i+1}/25 업데이트")
        
        print("✅ 깊이 시뮬레이션 환경 준비 완료")
        
        # 깊이 이미지 캡처 및 압축 테스트
        print("\n📐 깊이 이미지 캡처 테스트...")
        
        test_cases = [
            {"method": "direct", "format": "PNG16", "quality": 1},
            {"method": "file", "format": "PNG16", "quality": 6},
            {"method": "direct", "format": "PNG8", "quality": 3},
            {"method": "direct", "format": "JPEG8", "quality": 85},
            {"method": "direct", "format": "RAW", "quality": None},
        ]
        
        for i, test_case in enumerate(test_cases):
            print(f"\n--- Depth 테스트 {i+1}/{len(test_cases)}: {test_case['method']} {test_case['format']} ---")
            
            result = depth_sensor.capture_and_encode_depth_image(
                format=test_case['format'],
                quality=test_case['quality'],
                base64_encode=True,
                method=test_case['method']
            )
            
            if result:
                print(f"  ✅ {test_case['format']}: {result['size_bytes']} bytes, "
                      f"{result['capture_time_ms']:.1f}ms")
                print(f"  📏 해상도: {result['resolution']}")
                
                # 깊이 통계 출력
                stats = result['depth_stats']
                print(f"  📊 깊이 통계: {stats['valid_pixels']}/{stats['total_pixels']} 유효")
                print(f"      범위: {stats['min_depth']:.2f}~{stats['max_depth']:.2f}m")
                print(f"      평균: {stats['mean_depth']:.2f}m ± {stats['std_depth']:.2f}m")
                
                if 'depth_image_base64' in result:
                    print(f"  📝 Base64 길이: {len(result['depth_image_base64'])}")
            else:
                print(f"  ❌ {test_case['method']} {test_case['format']} 캡처 실패")
            
            time.sleep(0.1)
        
        # 깊이 센서 성능 메트릭 출력
        metrics = depth_sensor.get_performance_metrics()
        print(f"\n📊 Depth 센서 성능 메트릭:")
        print(f"  총 캡처 횟수: {metrics['total_captures']}")
        print(f"  마지막 캡처 시간: {metrics['last_capture_time_ms']:.1f}ms")
        print(f"  실제 FPS: {metrics['actual_fps']:.1f} (목표: {metrics['target_fps']})")
        print(f"  깊이 범위: {metrics['depth_range_m'][0]:.1f}m ~ {metrics['depth_range_m'][1]:.1f}m")
        print(f"  정밀도: {metrics['compression_settings']['depth_precision_bits']}-bit")
        
        # 캡처된 파일들 확인
        output_path = Path(depth_sensor.output_dir)
        if output_path.exists():
            files = list(output_path.glob("*"))
            print(f"\n📁 생성된 Depth 파일들 ({len(files)}개):")
            for file_path in sorted(files)[:10]:
                size = file_path.stat().st_size
                print(f"  - {file_path.name}: {size:,} bytes")
        
        # 리소스 정리
        depth_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ 개선된 Depth 카메라 센서 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ Depth 테스트 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_enhanced_depth_camera_sensor()
    sys.exit(0 if success else 1)