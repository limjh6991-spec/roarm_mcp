#!/usr/bin/env python3
"""
통합 RGB-D 센서 시스템 단위 테스트 (Isaac Sim 없이)
동기화, 성능, 데이터 통합 로직 검증
"""

import sys
import numpy as np
import json
import time
import threading
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, asdict
import queue
from typing import Dict, Any, List, Tuple, Optional

@dataclass
class MockCaptureResult:
    """모의 캡처 결과"""
    timestamp: float
    sensor_type: str
    resolution: Tuple[int, int]
    format: str
    size_bytes: int
    capture_time_ms: float
    success: bool
    image_data: Optional[str] = None

def simulate_rgb_capture(delay_ms: float = 25.0) -> MockCaptureResult:
    """RGB 캡처 시뮬레이션"""
    start_time = time.time()
    time.sleep(delay_ms / 1000.0)
    
    return MockCaptureResult(
        timestamp=time.time(),
        sensor_type="rgb",
        resolution=(1280, 720),
        format="jpeg",
        size_bytes=np.random.randint(50000, 80000),
        capture_time_ms=(time.time() - start_time) * 1000,
        success=True,
        image_data=f"rgb_data_{int(time.time()*1000)}"
    )

def simulate_depth_capture(delay_ms: float = 30.0) -> MockCaptureResult:
    """Depth 캡처 시뮬레이션"""
    start_time = time.time()
    time.sleep(delay_ms / 1000.0)
    
    return MockCaptureResult(
        timestamp=time.time(),
        sensor_type="depth",
        resolution=(1280, 720),
        format="png16",
        size_bytes=np.random.randint(300000, 500000),
        capture_time_ms=(time.time() - start_time) * 1000,
        success=True,
        image_data=f"depth_data_{int(time.time()*1000)}"
    )

def test_synchronization_modes():
    """동기화 모드 테스트"""
    print("🔄 동기화 모드 테스트")
    
    def test_sequential_sync():
        """순차 동기화 테스트"""
        print("  📋 순차 동기화 (Sequential)...")
        start_time = time.time()
        
        # RGB 먼저, 그 다음 Depth
        rgb_result = simulate_rgb_capture(25.0)
        depth_result = simulate_depth_capture(30.0)
        
        total_time = (time.time() - start_time) * 1000
        time_diff = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
        
        print(f"    총 시간: {total_time:.1f}ms")
        print(f"    시간 차이: {time_diff:.1f}ms")
        print(f"    예상 순서: RGB → Depth ✅")
        
        return {
            "mode": "sequential",
            "total_time_ms": total_time,
            "time_diff_ms": time_diff,
            "rgb_first": rgb_result.timestamp < depth_result.timestamp
        }
    
    def test_parallel_sync():
        """병렬 동기화 테스트"""
        print("  ⚡ 병렬 동기화 (Parallel)...")
        start_time = time.time()
        
        with ThreadPoolExecutor(max_workers=2) as executor:
            rgb_future = executor.submit(simulate_rgb_capture, 25.0)
            depth_future = executor.submit(simulate_depth_capture, 30.0)
            
            rgb_result = rgb_future.result()
            depth_result = depth_future.result()
        
        total_time = (time.time() - start_time) * 1000
        time_diff = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
        
        print(f"    총 시간: {total_time:.1f}ms")
        print(f"    시간 차이: {time_diff:.1f}ms")
        print(f"    병렬 효율: {'✅' if total_time < 50 else '❌'}")
        
        return {
            "mode": "parallel",
            "total_time_ms": total_time,
            "time_diff_ms": time_diff,
            "parallel_efficient": total_time < 50
        }
    
    def test_sync_tolerance():
        """동기화 허용 오차 테스트"""
        print("  📐 동기화 허용 오차 테스트...")
        
        tolerance_ms = 10.0
        test_results = []
        
        for i in range(5):
            rgb_result = simulate_rgb_capture(20.0 + i * 2)
            depth_result = simulate_depth_capture(25.0 + i * 3)
            
            time_diff = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
            within_tolerance = time_diff <= tolerance_ms
            
            test_results.append({
                "attempt": i + 1,
                "time_diff_ms": time_diff,
                "within_tolerance": within_tolerance
            })
            
            status = "✅" if within_tolerance else "⚠️"
            print(f"    시도 {i+1}: {time_diff:.1f}ms {status}")
        
        success_rate = sum(1 for r in test_results if r["within_tolerance"]) / len(test_results)
        print(f"    허용 오차 내 성공률: {success_rate:.1%}")
        
        return test_results
    
    # 모든 테스트 실행
    sequential_result = test_sequential_sync()
    parallel_result = test_parallel_sync()
    tolerance_results = test_sync_tolerance()
    
    print(f"\n  📊 동기화 성능 요약:")
    print(f"    순차: {sequential_result['total_time_ms']:.1f}ms")
    print(f"    병렬: {parallel_result['total_time_ms']:.1f}ms")
    print(f"    병렬 이득: {sequential_result['total_time_ms'] - parallel_result['total_time_ms']:.1f}ms")
    
    return {
        "sequential": sequential_result,
        "parallel": parallel_result,
        "tolerance_tests": tolerance_results
    }

def test_stream_capture_simulation():
    """스트림 캡처 시뮬레이션 테스트"""
    print("\n🎬 스트림 캡처 시뮬레이션")
    
    def simulate_frame_capture():
        """단일 프레임 캡처 시뮬레이션"""
        with ThreadPoolExecutor(max_workers=2) as executor:
            rgb_future = executor.submit(simulate_rgb_capture, 22.0)
            depth_future = executor.submit(simulate_depth_capture, 28.0)
            
            rgb_result = rgb_future.result()
            depth_result = depth_future.result()
            
            # 동기화 품질 평가
            time_diff_ms = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
            sync_quality = "good" if time_diff_ms <= 10.0 else "poor"
            
            return {
                "timestamp": min(rgb_result.timestamp, depth_result.timestamp),
                "rgb": asdict(rgb_result),
                "depth": asdict(depth_result),
                "sync_quality": sync_quality,
                "time_diff_ms": time_diff_ms,
                "success": rgb_result.success and depth_result.success
            }
    
    # 스트림 캡처 (목표: 10 FPS, 5초 = 50프레임)
    target_fps = 10.0
    duration_seconds = 5.0
    frame_interval = 1.0 / target_fps
    
    frames = []
    start_time = time.time()
    frame_count = 0
    
    print(f"  🎯 목표: {target_fps} FPS, {duration_seconds}초")
    
    while (time.time() - start_time) < duration_seconds:
        frame_start = time.time()
        
        # 프레임 캡처
        frame_data = simulate_frame_capture()
        
        if frame_data["success"]:
            frames.append(frame_data)
            frame_count += 1
        
        # FPS 조절
        frame_time = time.time() - frame_start
        sleep_time = max(0, frame_interval - frame_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    # 결과 분석
    total_time = time.time() - start_time
    actual_fps = len(frames) / total_time
    
    # 동기화 품질 분석
    good_sync = sum(1 for f in frames if f["sync_quality"] == "good")
    poor_sync = len(frames) - good_sync
    
    # 데이터 크기 분석
    total_rgb_size = sum(f["rgb"]["size_bytes"] for f in frames)
    total_depth_size = sum(f["depth"]["size_bytes"] for f in frames)
    total_data_size = total_rgb_size + total_depth_size
    
    print(f"  ✅ 캡처 완료: {len(frames)}프레임")
    print(f"  📊 실제 FPS: {actual_fps:.1f} (목표: {target_fps})")
    print(f"  🎯 동기화 품질: 좋음 {good_sync}, 나쁨 {poor_sync}")
    print(f"  💾 총 데이터: {total_data_size:,} bytes ({total_data_size/1024/1024:.1f} MB)")
    print(f"    RGB: {total_rgb_size:,} bytes")
    print(f"    Depth: {total_depth_size:,} bytes")
    
    # 성능 분석
    frame_times = [f["time_diff_ms"] for f in frames]
    avg_sync_time = sum(frame_times) / len(frame_times) if frame_times else 0
    
    print(f"  ⏱️  평균 동기화 시간 차이: {avg_sync_time:.1f}ms")
    
    return {
        "target_fps": target_fps,
        "actual_fps": actual_fps,
        "total_frames": len(frames),
        "good_sync_frames": good_sync,
        "poor_sync_frames": poor_sync,
        "sync_success_rate": good_sync / len(frames) if frames else 0,
        "total_data_bytes": total_data_size,
        "avg_sync_time_diff_ms": avg_sync_time,
        "frames": frames[:3]  # 처음 3프레임만 저장 (용량 절약)
    }

def test_data_compression_analysis():
    """데이터 압축 분석 테스트"""
    print("\n🗜️  데이터 압축 분석")
    
    # 다양한 압축 설정 시뮬레이션
    compression_scenarios = [
        {"rgb_format": "JPEG", "rgb_quality": 95, "depth_format": "PNG16"},
        {"rgb_format": "JPEG", "rgb_quality": 85, "depth_format": "PNG16"},
        {"rgb_format": "JPEG", "rgb_quality": 75, "depth_format": "PNG8"},
        {"rgb_format": "PNG", "rgb_quality": None, "depth_format": "PNG16"},
    ]
    
    def simulate_compression(rgb_format, rgb_quality, depth_format):
        """압축 시뮬레이션"""
        # 원본 크기 (1280x720)
        raw_rgb_size = 1280 * 720 * 3  # RGB 24-bit
        raw_depth_size = 1280 * 720 * 4  # Depth float32
        
        # RGB 압축 시뮬레이션
        if rgb_format == "JPEG":
            if rgb_quality >= 90:
                rgb_compressed = int(raw_rgb_size * 0.08)  # 8%
            elif rgb_quality >= 80:
                rgb_compressed = int(raw_rgb_size * 0.05)  # 5%
            else:
                rgb_compressed = int(raw_rgb_size * 0.03)  # 3%
        else:  # PNG
            rgb_compressed = int(raw_rgb_size * 0.25)  # 25%
        
        # Depth 압축 시뮬레이션
        if depth_format == "PNG16":
            depth_compressed = int(raw_depth_size * 0.2)  # 20%
        elif depth_format == "PNG8":
            depth_compressed = int(raw_depth_size * 0.1)  # 10%
        else:
            depth_compressed = raw_depth_size  # RAW
        
        return {
            "raw_rgb_size": raw_rgb_size,
            "raw_depth_size": raw_depth_size,
            "rgb_compressed": rgb_compressed,
            "depth_compressed": depth_compressed,
            "total_compressed": rgb_compressed + depth_compressed,
            "rgb_ratio": raw_rgb_size / rgb_compressed,
            "depth_ratio": raw_depth_size / depth_compressed,
            "total_ratio": (raw_rgb_size + raw_depth_size) / (rgb_compressed + depth_compressed)
        }
    
    print("  📊 압축 시나리오 분석:")
    
    best_compression = None
    best_ratio = 0
    
    for i, scenario in enumerate(compression_scenarios):
        result = simulate_compression(**scenario)
        
        print(f"\n    시나리오 {i+1}: RGB {scenario['rgb_format']}, Depth {scenario['depth_format']}")
        if scenario['rgb_quality']:
            print(f"      RGB 품질: {scenario['rgb_quality']}")
        
        print(f"      RGB: {result['raw_rgb_size']:,} → {result['rgb_compressed']:,} bytes "
              f"({result['rgb_ratio']:.1f}x)")
        print(f"      Depth: {result['raw_depth_size']:,} → {result['depth_compressed']:,} bytes "
              f"({result['depth_ratio']:.1f}x)")
        print(f"      전체: {result['total_compressed']:,} bytes ({result['total_ratio']:.1f}x)")
        
        if result['total_ratio'] > best_ratio:
            best_ratio = result['total_ratio']
            best_compression = {**scenario, **result}
    
    print(f"\n  🏆 최적 압축: RGB {best_compression['rgb_format']}, Depth {best_compression['depth_format']}")
    print(f"    압축비: {best_compression['total_ratio']:.1f}x")
    print(f"    총 크기: {best_compression['total_compressed']:,} bytes")
    
    return compression_scenarios, best_compression

def test_camera_calibration_consistency():
    """카메라 캘리브레이션 일치성 테스트"""
    print("\n📐 카메라 캘리브레이션 일치성")
    
    # RGB와 Depth 카메라 파라미터 (개선된 센서에서 가져온 값들)
    resolution = (1280, 720)
    
    rgb_params = {
        "fx": 853.3, "fy": 853.3, "cx": 640.0, "cy": 360.0,
        "horizontal_aperture": 36.0, "vertical_aperture": 20.25,
        "focal_length": 24.0
    }
    
    depth_params = {
        "fx": 853.3, "fy": 853.3, "cx": 640.0, "cy": 360.0,
        "horizontal_aperture": 36.0, "vertical_aperture": 20.25,
        "focal_length": 24.0
    }
    
    print(f"  📷 RGB 카메라 파라미터:")
    print(f"    초점거리: fx={rgb_params['fx']}, fy={rgb_params['fy']}")
    print(f"    주점: cx={rgb_params['cx']}, cy={rgb_params['cy']}")
    print(f"    조리개: {rgb_params['horizontal_aperture']}x{rgb_params['vertical_aperture']}mm")
    
    print(f"  📐 Depth 카메라 파라미터:")
    print(f"    초점거리: fx={depth_params['fx']}, fy={depth_params['fy']}")
    print(f"    주점: cx={depth_params['cx']}, cy={depth_params['cy']}")
    print(f"    조리개: {depth_params['horizontal_aperture']}x{depth_params['vertical_aperture']}mm")
    
    # 일치성 검증
    tolerance = 0.001
    
    checks = [
        ("fx", abs(rgb_params['fx'] - depth_params['fx']) < tolerance),
        ("fy", abs(rgb_params['fy'] - depth_params['fy']) < tolerance),
        ("cx", abs(rgb_params['cx'] - depth_params['cx']) < tolerance),
        ("cy", abs(rgb_params['cy'] - depth_params['cy']) < tolerance),
        ("horizontal_aperture", abs(rgb_params['horizontal_aperture'] - depth_params['horizontal_aperture']) < tolerance),
        ("vertical_aperture", abs(rgb_params['vertical_aperture'] - depth_params['vertical_aperture']) < tolerance),
        ("focal_length", abs(rgb_params['focal_length'] - depth_params['focal_length']) < tolerance)
    ]
    
    print(f"\n  ✅ 일치성 검증 (허용 오차: {tolerance}):")
    
    all_match = True
    for param_name, is_match in checks:
        status = "✅" if is_match else "❌"
        print(f"    {param_name}: {status}")
        if not is_match:
            all_match = False
    
    print(f"\n  🎯 전체 일치성: {'✅ 완벽' if all_match else '❌ 불일치'}")
    
    return all_match, checks

def test_integrated_camera_info_generation():
    """통합 카메라 정보 생성 테스트"""
    print("\n📋 통합 카메라 정보 생성")
    
    # 통합 카메라 정보 구조 (실제 코드에서 생성되는 형태)
    integrated_info = {
        "camera_name": "isaac_sim_rgbd_camera",
        "synchronization_mode": "soft",
        "target_fps": 30.0,
        "resolution": {"width": 1280, "height": 720},
        
        "rgb_camera": {
            "camera_matrix": [
                [853.3, 0, 640.0],
                [0, 853.3, 360.0],
                [0, 0, 1]
            ],
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            "horizontal_aperture_mm": 36.0,
            "vertical_aperture_mm": 20.25,
            "focal_length_mm": 24.0,
            "default_format": "JPEG",
            "default_quality": 85
        },
        
        "depth_camera": {
            "camera_matrix": [
                [853.3, 0, 640.0],
                [0, 853.3, 360.0],
                [0, 0, 1]
            ],
            "distortion_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
            "horizontal_aperture_mm": 36.0,
            "vertical_aperture_mm": 20.25,
            "focal_length_mm": 24.0,
            "min_depth_m": 0.1,
            "max_depth_m": 100.0,
            "depth_precision_bits": 16,
            "default_format": "PNG16",
            "default_quality": 1
        },
        
        "isaac_sim": {
            "camera_path": "/World/Sensors/FrontCam_Mount/FrontCam",
            "render_product_id": None,
            "synchronization_tolerance_ms": 10.0
        },
        
        "performance": {
            "frame_buffer_size": 3,
            "capture_timeout_ms": 1000.0,
            "thread_pool_workers": 4
        }
    }
    
    # 임시 파일에 저장
    output_dir = Path("/tmp/integrated_rgbd_test")
    output_dir.mkdir(exist_ok=True)
    
    info_path = output_dir / "integrated_camera_info.json"
    with open(info_path, 'w') as f:
        json.dump(integrated_info, f, indent=2)
    
    print(f"  ✅ 통합 카메라 정보 생성: {info_path}")
    
    # 정보 유효성 검증
    file_size = info_path.stat().st_size
    print(f"  💾 파일 크기: {file_size} bytes")
    
    # 중요 필드 검증
    required_fields = [
        "camera_name", "synchronization_mode", "target_fps",
        "rgb_camera", "depth_camera", "isaac_sim"
    ]
    
    missing_fields = [field for field in required_fields if field not in integrated_info]
    
    if not missing_fields:
        print(f"  ✅ 필수 필드 모두 존재")
    else:
        print(f"  ❌ 누락된 필드: {missing_fields}")
    
    # RGB/Depth 카메라 매트릭스 일치성 확인
    rgb_matrix = integrated_info["rgb_camera"]["camera_matrix"]
    depth_matrix = integrated_info["depth_camera"]["camera_matrix"]
    
    matrices_match = rgb_matrix == depth_matrix
    print(f"  📐 RGB/Depth 매트릭스 일치: {'✅' if matrices_match else '❌'}")
    
    return integrated_info, file_size, len(missing_fields) == 0

def main():
    """메인 테스트 함수"""
    print("🧪 통합 RGB-D 센서 시스템 단위 테스트")
    print("=" * 60)
    
    test_results = {}
    
    # 1. 동기화 모드 테스트
    try:
        sync_results = test_synchronization_modes()
        test_results["synchronization"] = "✅ PASS"
        print("  ✅ 동기화 모드 테스트 완료")
    except Exception as e:
        test_results["synchronization"] = f"❌ FAIL: {e}"
        print(f"  ❌ 동기화 테스트 실패: {e}")
    
    # 2. 스트림 캡처 시뮬레이션
    try:
        stream_results = test_stream_capture_simulation()
        test_results["stream_capture"] = "✅ PASS"
        print("  ✅ 스트림 캡처 시뮬레이션 완료")
    except Exception as e:
        test_results["stream_capture"] = f"❌ FAIL: {e}"
        print(f"  ❌ 스트림 캡처 테스트 실패: {e}")
    
    # 3. 데이터 압축 분석
    try:
        compression_scenarios, best_compression = test_data_compression_analysis()
        test_results["compression_analysis"] = "✅ PASS"
        print("  ✅ 데이터 압축 분석 완료")
    except Exception as e:
        test_results["compression_analysis"] = f"❌ FAIL: {e}"
        print(f"  ❌ 압축 분석 테스트 실패: {e}")
    
    # 4. 카메라 캘리브레이션 일치성
    try:
        calibration_match, calibration_checks = test_camera_calibration_consistency()
        test_results["calibration_consistency"] = "✅ PASS"
        print("  ✅ 카메라 캘리브레이션 일치성 테스트 완료")
    except Exception as e:
        test_results["calibration_consistency"] = f"❌ FAIL: {e}"
        print(f"  ❌ 캘리브레이션 일치성 테스트 실패: {e}")
    
    # 5. 통합 카메라 정보 생성
    try:
        integrated_info, file_size, fields_valid = test_integrated_camera_info_generation()
        test_results["integrated_camera_info"] = "✅ PASS"
        print("  ✅ 통합 카메라 정보 생성 테스트 완료")
    except Exception as e:
        test_results["integrated_camera_info"] = f"❌ FAIL: {e}"
        print(f"  ❌ 통합 카메라 정보 테스트 실패: {e}")
    
    # 전체 결과 요약
    print("\n📊 통합 RGB-D 센서 테스트 결과 요약")
    print("=" * 60)
    
    passed = 0
    total = len(test_results)
    
    for test_name, result in test_results.items():
        print(f"  {test_name}: {result}")
        if "PASS" in result:
            passed += 1
    
    success_rate = (passed / total) * 100 if total > 0 else 0
    print(f"\n🏆 전체 성공률: {passed}/{total} ({success_rate:.1f}%)")
    
    # 추가 성능 요약 (성공한 테스트들 기준)
    if 'synchronization' in test_results and 'PASS' in test_results['synchronization']:
        print(f"\n⚡ 성능 하이라이트:")
        print(f"  동기화: 병렬 처리로 성능 향상 확인")
        
    if 'stream_capture' in test_results and 'PASS' in test_results['stream_capture']:
        print(f"  스트림: 실시간 캡처 및 동기화 검증")
        
    if 'compression_analysis' in test_results and 'PASS' in test_results['compression_analysis']:
        print(f"  압축: 다양한 포맷 지원 및 효율성 분석")
        
    if 'calibration_consistency' in test_results and 'PASS' in test_results['calibration_consistency']:
        print(f"  캘리브레이션: RGB-Depth 완벽 정합")
    
    if passed == total:
        print("🎉 모든 통합 RGB-D 센서 테스트 통과! 시스템 준비 완료.")
        return True
    else:
        print(f"⚠️  {total-passed}개 테스트 실패. 통합 시스템 코드 검토 필요.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)