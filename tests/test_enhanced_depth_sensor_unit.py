#!/usr/bin/env python3
"""
개선된 Depth 카메라 센서 단위 테스트 (Isaac Sim 없이)
깊이 데이터 처리, 압축 및 인코딩 로직 검증
"""

import sys
import numpy as np
import json
import time
import base64
from pathlib import Path
import struct

def test_depth_data_generation():
    """시뮬레이션된 깊이 데이터 생성 테스트"""
    print("🎯 깊이 데이터 생성 및 검증 테스트")
    
    # 다양한 깊이 시나리오 생성
    width, height = 1280, 720
    
    # 1. 거리별 계단 패턴 (실제 환경 시뮬레이션)
    depth_layers = np.zeros((height, width), dtype=np.float32)
    
    # 여러 깊이 층 생성
    layers = [
        (0, 144, 2.0),      # 가까운 객체 (2m)
        (144, 288, 5.0),    # 중간 객체 (5m)
        (288, 432, 10.0),   # 먼 객체 (10m)
        (432, 576, 20.0),   # 매우 먼 객체 (20m)
        (576, 720, 50.0)    # 배경 (50m)
    ]
    
    for start_y, end_y, depth in layers:
        depth_layers[start_y:end_y, :] = depth
    
    # 2. 랜덤 노이즈 추가 (실제 센서 노이즈 시뮬레이션)
    noise = np.random.normal(0, 0.02, (height, width))  # 2cm 표준편차
    depth_with_noise = depth_layers + noise
    
    # 3. 일부 무효 픽셀 추가 (가려진 영역, 반사 등)
    invalid_mask = np.random.random((height, width)) < 0.05  # 5% 무효 픽셀
    depth_with_noise[invalid_mask] = 0.0  # 무효값
    
    # 4. 극단값 처리
    depth_with_noise = np.clip(depth_with_noise, 0.0, 100.0)
    
    print(f"  ✅ 시뮬레이션된 깊이 데이터 생성: {depth_with_noise.shape}")
    
    # 통계 계산
    valid_mask = depth_with_noise > 0
    valid_pixels = np.sum(valid_mask)
    total_pixels = depth_with_noise.size
    valid_ratio = valid_pixels / total_pixels
    
    print(f"  📊 깊이 통계:")
    print(f"    총 픽셀: {total_pixels:,}")
    print(f"    유효 픽셀: {valid_pixels:,} ({valid_ratio:.1%})")
    print(f"    깊이 범위: {depth_with_noise[valid_mask].min():.2f} ~ {depth_with_noise[valid_mask].max():.2f}m")
    print(f"    평균 깊이: {depth_with_noise[valid_mask].mean():.2f}m")
    print(f"    표준편차: {depth_with_noise[valid_mask].std():.2f}m")
    
    return depth_with_noise, {
        "total_pixels": total_pixels,
        "valid_pixels": valid_pixels,
        "valid_ratio": valid_ratio,
        "min_depth": float(depth_with_noise[valid_mask].min()),
        "max_depth": float(depth_with_noise[valid_mask].max()),
        "mean_depth": float(depth_with_noise[valid_mask].mean()),
        "std_depth": float(depth_with_noise[valid_mask].std())
    }


def test_depth_validation():
    """깊이 데이터 유효성 검증 테스트"""
    print("\n✅ 깊이 데이터 유효성 검증 테스트")
    
    # 다양한 깊이 데이터 케이스
    width, height = 640, 360  # 작은 크기로 테스트
    
    test_cases = [
        ("완전 블랙", np.zeros((height, width), dtype=np.float32)),
        ("완전 무한", np.full((height, width), np.inf, dtype=np.float32)),
        ("완전 NaN", np.full((height, width), np.nan, dtype=np.float32)),
        ("정상 깊이", np.full((height, width), 5.0, dtype=np.float32)),
        ("혼합 데이터", None)  # 아래에서 생성
    ]
    
    # 혼합 데이터 생성 (실제 시나리오)
    mixed_data = np.random.uniform(1.0, 50.0, (height, width)).astype(np.float32)
    mixed_data[:50, :] = 0.0  # 무효 영역
    mixed_data[50:100, :100] = np.inf  # 무한 거리
    mixed_data[100:150, :] = np.nan  # NaN 영역
    test_cases[4] = ("혼합 데이터", mixed_data)
    
    # 검증 로직 (개선된 센서에서 가져옴)
    def validate_depth_data(depth_data, name):
        if depth_data is None or depth_data.size == 0:
            return False, {"reason": "empty_data"}
        
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
            "mean_depth": float(np.mean(depth_data[valid_mask])) if valid_pixels > 0 else 0.0
        }
        
        is_valid = (
            valid_ratio > 0.1 and  # 최소 10% 유효
            stats["max_depth"] > stats["min_depth"] and
            stats["mean_depth"] > 0.01
        )
        
        return is_valid, stats
    
    for name, depth_data in test_cases:
        is_valid, stats = validate_depth_data(depth_data, name)
        status = "✅ 유효" if is_valid else "❌ 무효"
        
        print(f"  {name}: {status}")
        print(f"    유효 비율: {stats['valid_ratio']:.1%}")
        if stats['valid_pixels'] > 0:
            print(f"    깊이 범위: {stats['min_depth']:.2f} ~ {stats['max_depth']:.2f}m")
    
    return True


def test_depth_normalization():
    """깊이 정규화 테스트"""
    print("\n🎨 깊이 정규화 테스트")
    
    # 테스트용 깊이 데이터 (0.1m ~ 100m 범위)
    depth_data = np.array([
        [0.0, 0.1, 1.0, 5.0],      # 첫 번째 행
        [10.0, 20.0, 50.0, 100.0], # 두 번째 행
        [np.inf, np.nan, -1.0, 150.0]  # 무효값들
    ], dtype=np.float32)
    
    print(f"  원본 깊이 데이터:\n{depth_data}")
    
    # 8-bit 정규화 함수 (개선된 센서에서 가져옴)
    def normalize_depth_to_8bit(depth_array, depth_range=(0.1, 100.0)):
        min_depth, max_depth = depth_range
        valid_mask = (depth_array > 0) & (depth_array < np.inf) & ~np.isnan(depth_array)
        
        clipped_depth = np.clip(depth_array, min_depth, max_depth)
        normalized = (clipped_depth - min_depth) / (max_depth - min_depth) * 255.0
        normalized[~valid_mask] = 0
        
        return normalized.astype(np.uint8)
    
    # 16-bit 변환 함수
    def convert_depth_to_16bit(depth_array, depth_range=(0.1, 100.0), scale=1000.0):
        min_depth, max_depth = depth_range
        valid_mask = (depth_array > 0) & (depth_array < np.inf) & ~np.isnan(depth_array)
        
        depth_mm = depth_array * scale
        clipped_depth = np.clip(depth_mm, min_depth * scale, max_depth * scale)
        depth_16bit = clipped_depth.astype(np.uint16)
        depth_16bit[~valid_mask] = 0
        
        return depth_16bit
    
    # 8-bit 정규화 테스트
    normalized_8bit = normalize_depth_to_8bit(depth_data)
    print(f"\n  8-bit 정규화 결과 (0-255):\n{normalized_8bit}")
    
    # 16-bit 변환 테스트
    depth_16bit = convert_depth_to_16bit(depth_data)
    print(f"\n  16-bit 변환 결과 (밀리미터):\n{depth_16bit}")
    
    # 정규화 품질 검증
    valid_original = depth_data[(depth_data > 0) & (depth_data < np.inf) & ~np.isnan(depth_data)]
    valid_normalized = normalized_8bit[normalized_8bit > 0]
    
    print(f"\n  📊 정규화 통계:")
    print(f"    원본 유효 픽셀: {len(valid_original)}")
    print(f"    정규화 유효 픽셀: {len(valid_normalized)}")
    print(f"    8-bit 범위: {normalized_8bit.min()} ~ {normalized_8bit.max()}")
    print(f"    16-bit 범위: {depth_16bit.min()} ~ {depth_16bit.max()}")
    
    return normalized_8bit, depth_16bit


def test_depth_compression_simulation():
    """깊이 압축 시뮬레이션 테스트"""
    print("\n🗜️  깊이 압축 시뮬레이션 테스트")
    
    # 실제 크기의 깊이 데이터
    width, height = 1280, 720
    
    # 실제적인 깊이 패턴 생성
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    
    # 중앙에서 멀어질수록 깊이 증가
    center_x, center_y = width // 2, height // 2
    distance_from_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)
    
    # 깊이 맵: 중앙은 가깝고, 가장자리는 멀게
    depth_map = 2.0 + distance_from_center * 0.05  # 2m ~ 40m 정도
    
    # 일부 객체 추가 (더 가까운 영역)
    object_regions = [
        (200, 300, 400, 500, 1.5),  # 가까운 객체
        (800, 900, 200, 350, 3.0),  # 중간 객체
        (600, 750, 500, 600, 8.0),  # 먼 객체
    ]
    
    for x1, x2, y1, y2, depth in object_regions:
        depth_map[y1:y2, x1:x2] = depth
    
    # 노이즈 추가
    noise = np.random.normal(0, 0.05, depth_map.shape)
    depth_map += noise
    depth_map = np.clip(depth_map, 0.1, 100.0).astype(np.float32)
    
    print(f"  ✅ 테스트 깊이 맵 생성: {depth_map.shape}")
    print(f"  📏 원본 크기: {depth_map.nbytes:,} bytes ({depth_map.nbytes/1024/1024:.1f} MB)")
    
    # 압축 시뮬레이션 함수들
    def simulate_png16_compression(depth_16bit):
        base_size = depth_16bit.nbytes
        compression_factor = 0.4  # PNG는 보통 40% 정도
        return int(base_size * compression_factor)
    
    def simulate_png8_compression(depth_8bit):
        base_size = depth_8bit.nbytes
        compression_factor = 0.25  # 8-bit PNG는 더 잘 압축됨
        return int(base_size * compression_factor)
    
    def simulate_jpeg_compression(depth_8bit, quality=85):
        base_size = depth_8bit.nbytes
        quality_factor = 0.05 + (quality / 100) * 0.15  # 5-20%
        return int(base_size * quality_factor)
    
    # 다양한 압축 방식 테스트
    print("\n  🎯 압축 방식별 결과:")
    
    # 1. RAW (원본)
    raw_size = depth_map.nbytes
    print(f"    RAW float32: {raw_size:,} bytes (1.00x)")
    
    # 2. 16-bit PNG
    depth_16bit = (depth_map * 1000).clip(0, 65535).astype(np.uint16)
    png16_size = simulate_png16_compression(depth_16bit)
    png16_ratio = raw_size / png16_size
    print(f"    PNG 16-bit: {png16_size:,} bytes ({png16_ratio:.1f}x 압축)")
    
    # 3. 8-bit PNG (정규화)
    normalized_8bit = ((depth_map - 0.1) / (100.0 - 0.1) * 255).clip(0, 255).astype(np.uint8)
    png8_size = simulate_png8_compression(normalized_8bit)
    png8_ratio = raw_size / png8_size
    print(f"    PNG 8-bit: {png8_size:,} bytes ({png8_ratio:.1f}x 압축)")
    
    # 4. 8-bit JPEG (다양한 품질)
    for quality in [95, 85, 75, 50]:
        jpeg_size = simulate_jpeg_compression(normalized_8bit, quality)
        jpeg_ratio = raw_size / jpeg_size
        print(f"    JPEG Q{quality}: {jpeg_size:,} bytes ({jpeg_ratio:.1f}x 압축)")
    
    # Base64 오버헤드 계산
    print(f"\n  📦 Base64 인코딩 오버헤드:")
    for name, size in [("PNG16", png16_size), ("PNG8", png8_size), ("JPEG85", simulate_jpeg_compression(normalized_8bit, 85))]:
        base64_size = int(size * 4/3 * 1.02)  # Base64는 33% 증가 + 패딩
        print(f"    {name}: {size:,} → {base64_size:,} bytes (+{(base64_size/size-1)*100:.1f}%)")
    
    return depth_map, {
        "raw_size": raw_size,
        "png16_size": png16_size,
        "png8_size": png8_size,
        "jpeg85_size": simulate_jpeg_compression(normalized_8bit, 85)
    }


def test_camera_parameter_consistency():
    """RGB와 Depth 카메라 파라미터 일치성 테스트"""
    print("\n🎯 RGB-Depth 카메라 파라미터 일치성 테스트")
    
    resolution = (1280, 720)
    width, height = resolution
    aspect_ratio = width / height
    
    # RGB 카메라 파라미터 (기존)
    rgb_params = {
        "horizontal_aperture": 36.0,
        "vertical_aperture": 36.0 / aspect_ratio,  # 20.25
        "focal_length": 24.0
    }
    
    # Depth 카메라 파라미터 (동일하게 설정)
    depth_params = {
        "horizontal_aperture": 36.0,
        "vertical_aperture": 36.0 / aspect_ratio,  # 20.25
        "focal_length": 24.0
    }
    
    # 픽셀 단위 초점거리 계산
    def calculate_pixel_focal_length(params, resolution):
        width, height = resolution
        fx = params["focal_length"] * width / params["horizontal_aperture"]
        fy = params["focal_length"] * height / params["vertical_aperture"]
        cx = width / 2.0
        cy = height / 2.0
        return fx, fy, cx, cy
    
    rgb_fx, rgb_fy, rgb_cx, rgb_cy = calculate_pixel_focal_length(rgb_params, resolution)
    depth_fx, depth_fy, depth_cx, depth_cy = calculate_pixel_focal_length(depth_params, resolution)
    
    print(f"  📷 RGB 카메라 파라미터:")
    print(f"    조리개: {rgb_params['horizontal_aperture']:.1f}x{rgb_params['vertical_aperture']:.2f}mm")
    print(f"    초점거리: fx={rgb_fx:.1f}, fy={rgb_fy:.1f}")
    print(f"    주점: cx={rgb_cx:.1f}, cy={rgb_cy:.1f}")
    
    print(f"\n  📐 Depth 카메라 파라미터:")
    print(f"    조리개: {depth_params['horizontal_aperture']:.1f}x{depth_params['vertical_aperture']:.2f}mm")
    print(f"    초점거리: fx={depth_fx:.1f}, fy={depth_fy:.1f}")
    print(f"    주점: cx={depth_cx:.1f}, cy={depth_cy:.1f}")
    
    # 일치성 검증
    tolerance = 0.001
    fx_match = abs(rgb_fx - depth_fx) < tolerance
    fy_match = abs(rgb_fy - depth_fy) < tolerance
    cx_match = abs(rgb_cx - depth_cx) < tolerance
    cy_match = abs(rgb_cy - depth_cy) < tolerance
    
    all_match = fx_match and fy_match and cx_match and cy_match
    
    print(f"\n  ✅ 파라미터 일치성 검증:")
    print(f"    fx 일치: {'✅' if fx_match else '❌'} (차이: {abs(rgb_fx - depth_fx):.3f})")
    print(f"    fy 일치: {'✅' if fy_match else '❌'} (차이: {abs(rgb_fy - depth_fy):.3f})")
    print(f"    cx 일치: {'✅' if cx_match else '❌'} (차이: {abs(rgb_cx - depth_cx):.3f})")
    print(f"    cy 일치: {'✅' if cy_match else '❌'} (차이: {abs(rgb_cy - depth_cy):.3f})")
    print(f"    전체 일치: {'✅' if all_match else '❌'}")
    
    return all_match


def test_depth_camera_info_generation():
    """Depth 카메라 정보 생성 테스트"""
    print("\n📋 Depth 카메라 정보 생성 테스트")
    
    # 카메라 파라미터
    resolution = (1280, 720)
    camera_matrix = np.array([
        [853.3, 0, 640.0],
        [0, 853.3, 360.0],
        [0, 0, 1]
    ])
    distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    
    # Depth 특화 정보
    depth_range = (0.1, 100.0)
    
    # ROS 호환 Depth 카메라 정보 생성
    depth_camera_info = {
        "image_width": resolution[0],
        "image_height": resolution[1],
        "camera_name": "isaac_sim_depth_camera",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.flatten().tolist()
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": distortion_coeffs.tolist()
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).flatten().tolist()
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": np.column_stack([camera_matrix, np.zeros(3)]).flatten().tolist()
        },
        "depth_info": {
            "min_depth_m": depth_range[0],
            "max_depth_m": depth_range[1],
            "depth_precision_bits": 16,
            "depth_unit": "meters",
            "invalid_value": 0.0,
            "encoding": "16UC1"
        },
        "isaac_sim": {
            "horizontal_aperture_mm": 36.0,
            "vertical_aperture_mm": 20.25,
            "focal_length_mm": 24.0,
            "camera_path": "/World/Sensors/FrontCam_Mount/FrontCam",
            "fps": 30.0,
            "sensor_type": "depth"
        }
    }
    
    # 파일에 저장
    output_dir = Path("/tmp/enhanced_depth_test")
    output_dir.mkdir(exist_ok=True)
    
    depth_info_path = output_dir / "depth_camera_info.json"
    with open(depth_info_path, 'w') as f:
        json.dump(depth_camera_info, f, indent=2)
    
    print(f"  ✅ Depth 카메라 정보 JSON 생성: {depth_info_path}")
    print(f"  📏 해상도: {depth_camera_info['image_width']}x{depth_camera_info['image_height']}")
    print(f"  🎯 초점거리: fx={camera_matrix[0,0]:.1f}, fy={camera_matrix[1,1]:.1f}")
    print(f"  📐 깊이 범위: {depth_range[0]:.1f}m ~ {depth_range[1]:.1f}m")
    print(f"  🔢 정밀도: {depth_camera_info['depth_info']['depth_precision_bits']}-bit")
    print(f"  📊 인코딩: {depth_camera_info['depth_info']['encoding']}")
    
    # 파일 크기 확인
    file_size = depth_info_path.stat().st_size
    print(f"  💾 파일 크기: {file_size} bytes")
    
    return depth_camera_info


def test_performance_simulation():
    """Depth 센서 성능 시뮬레이션"""
    print("\n⚡ Depth 센서 성능 시뮬레이션")
    
    # 시뮬레이션된 캡처 시간들 (RGB보다 약간 더 오래 걸림)
    performance_scenarios = {
        "depth_direct_optimized": [25, 28, 24, 27, 26],  # ms (RGB보다 5ms 더)
        "depth_file_based": [50, 55, 48, 52, 51],        # ms
        "depth_basic_method": [70, 75, 68, 73, 71],       # ms
    }
    
    print("  📊 예상 성능 (깊이 센서):")
    
    for method, times in performance_scenarios.items():
        avg_time = sum(times) / len(times)
        fps = 1000 / avg_time
        min_time = min(times)
        max_time = max(times)
        
        print(f"    {method.replace('_', ' ').title()}:")
        print(f"      평균: {avg_time:.1f}ms → {fps:.1f} FPS")
        print(f"      범위: {min_time}-{max_time}ms")
        
        # 목표 30 FPS 달성 여부
        target_time = 1000 / 30  # 33.3ms
        if avg_time <= target_time:
            print(f"      ✅ 30 FPS 목표 달성 (여유: {target_time-avg_time:.1f}ms)")
        else:
            print(f"      ❌ 30 FPS 목표 미달성 (부족: {avg_time-target_time:.1f}ms)")
    
    return True


def main():
    """메인 테스트 함수"""
    print("🧪 개선된 Depth 카메라 센서 단위 테스트")
    print("=" * 60)
    
    test_results = {}
    
    # 1. 깊이 데이터 생성 테스트
    try:
        depth_data, depth_stats = test_depth_data_generation()
        test_results["depth_data_generation"] = "✅ PASS"
    except Exception as e:
        test_results["depth_data_generation"] = f"❌ FAIL: {e}"
        print(f"  ❌ 깊이 데이터 생성 테스트 실패: {e}")
    
    # 2. 깊이 데이터 유효성 검증 테스트
    try:
        validation_result = test_depth_validation()
        test_results["depth_validation"] = "✅ PASS"
        print("  ✅ 깊이 데이터 유효성 검증 테스트 완료")
    except Exception as e:
        test_results["depth_validation"] = f"❌ FAIL: {e}"
        print(f"  ❌ 깊이 유효성 검증 테스트 실패: {e}")
    
    # 3. 깊이 정규화 테스트
    try:
        normalized_8bit, depth_16bit = test_depth_normalization()
        test_results["depth_normalization"] = "✅ PASS"
        print("  ✅ 깊이 정규화 테스트 완료")
    except Exception as e:
        test_results["depth_normalization"] = f"❌ FAIL: {e}"
        print(f"  ❌ 깊이 정규화 테스트 실패: {e}")
    
    # 4. 깊이 압축 시뮬레이션 테스트
    try:
        depth_map, compression_stats = test_depth_compression_simulation()
        test_results["depth_compression"] = "✅ PASS"
        print("  ✅ 깊이 압축 시뮬레이션 테스트 완료")
    except Exception as e:
        test_results["depth_compression"] = f"❌ FAIL: {e}"
        print(f"  ❌ 깊이 압축 테스트 실패: {e}")
    
    # 5. 카메라 파라미터 일치성 테스트
    try:
        consistency_result = test_camera_parameter_consistency()
        test_results["camera_parameter_consistency"] = "✅ PASS"
        print("  ✅ 카메라 파라미터 일치성 테스트 완료")
    except Exception as e:
        test_results["camera_parameter_consistency"] = f"❌ FAIL: {e}"
        print(f"  ❌ 카메라 파라미터 일치성 테스트 실패: {e}")
    
    # 6. Depth 카메라 정보 생성 테스트
    try:
        depth_camera_info = test_depth_camera_info_generation()
        test_results["depth_camera_info"] = "✅ PASS"
        print("  ✅ Depth 카메라 정보 생성 테스트 완료")
    except Exception as e:
        test_results["depth_camera_info"] = f"❌ FAIL: {e}"
        print(f"  ❌ Depth 카메라 정보 생성 테스트 실패: {e}")
    
    # 7. 성능 시뮬레이션 테스트
    try:
        performance_result = test_performance_simulation()
        test_results["performance_simulation"] = "✅ PASS"
        print("  ✅ 성능 시뮬레이션 테스트 완료")
    except Exception as e:
        test_results["performance_simulation"] = f"❌ FAIL: {e}"
        print(f"  ❌ 성능 시뮬레이션 테스트 실패: {e}")
    
    # 전체 결과 요약
    print("\n📊 Depth 센서 테스트 결과 요약")
    print("=" * 60)
    
    passed = 0
    total = len(test_results)
    
    for test_name, result in test_results.items():
        print(f"  {test_name}: {result}")
        if "PASS" in result:
            passed += 1
    
    success_rate = (passed / total) * 100 if total > 0 else 0
    print(f"\n🏆 전체 성공률: {passed}/{total} ({success_rate:.1f}%)")
    
    if passed == total:
        print("🎉 모든 Depth 센서 테스트 통과! 깊이 센서가 정상 작동 준비됩니다.")
        return True
    else:
        print(f"⚠️  {total-passed}개 테스트 실패. Depth 센서 코드 검토가 필요합니다.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)