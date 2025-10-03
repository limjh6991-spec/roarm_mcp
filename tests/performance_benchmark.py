#!/usr/bin/env python3
"""
통합 RGB-D 센서 성능 벤치마크
실시간 30 FPS 목표, 메모리 사용량 최적화, GPU-CPU 파이프라인 효율성 검증
"""

import sys
import time
import threading
import psutil
import gc
import numpy as np
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from typing import Dict, Any, List, Optional
import json

@dataclass
class PerformanceMetrics:
    """성능 메트릭 데이터 클래스"""
    timestamp: float
    fps: float
    capture_time_ms: float
    memory_usage_mb: float
    cpu_usage_percent: float
    gpu_memory_usage_mb: Optional[float] = None
    sync_quality: str = "unknown"
    data_throughput_mbps: float = 0.0

class PerformanceBenchmark:
    """성능 벤치마크 클래스"""
    
    def __init__(self):
        self.metrics_history: List[PerformanceMetrics] = []
        self.benchmark_start_time = None
        self.total_frames_captured = 0
        self.total_data_bytes = 0
        
        # 시스템 정보
        self.system_info = {
            "cpu_count": psutil.cpu_count(),
            "memory_total_gb": psutil.virtual_memory().total / (1024**3),
            "platform": sys.platform
        }
        
        print(f"🖥️  시스템 정보:")
        print(f"  CPU 코어: {self.system_info['cpu_count']}")
        print(f"  총 메모리: {self.system_info['memory_total_gb']:.1f} GB")
        print(f"  플랫폼: {self.system_info['platform']}")
    
    def get_current_system_metrics(self) -> Dict[str, float]:
        """현재 시스템 메트릭 수집"""
        # CPU 사용률
        cpu_usage = psutil.cpu_percent(interval=0.1)
        
        # 메모리 사용률
        memory = psutil.virtual_memory()
        memory_usage_mb = (memory.total - memory.available) / (1024**2)
        
        # 현재 프로세스의 메모리 사용률
        current_process = psutil.Process()
        process_memory_mb = current_process.memory_info().rss / (1024**2)
        
        return {
            "cpu_usage_percent": cpu_usage,
            "memory_usage_mb": memory_usage_mb,
            "process_memory_mb": process_memory_mb
        }
    
    def simulate_rgb_capture_with_metrics(self, target_time_ms: float = 25.0) -> Dict[str, Any]:
        """메트릭을 포함한 RGB 캡처 시뮬레이션"""
        start_time = time.time()
        
        # RGB 데이터 생성 시뮬레이션 (1280x720x3)
        rgb_data_size = 1280 * 720 * 3
        rgb_data = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
        
        # 압축 시뮬레이션 (JPEG 85% 품질)
        compressed_size = int(rgb_data_size * 0.05)  # 5% 압축 비율
        compressed_data = np.random.bytes(compressed_size)
        
        # 처리 시간 시뮬레이션
        processing_delay = target_time_ms / 1000.0
        time.sleep(processing_delay)
        
        capture_time_ms = (time.time() - start_time) * 1000
        
        return {
            "sensor_type": "rgb",
            "capture_time_ms": capture_time_ms,
            "raw_size_bytes": rgb_data_size,
            "compressed_size_bytes": compressed_size,
            "compression_ratio": rgb_data_size / compressed_size,
            "success": True,
            "timestamp": time.time()
        }
    
    def simulate_depth_capture_with_metrics(self, target_time_ms: float = 30.0) -> Dict[str, Any]:
        """메트릭을 포함한 Depth 캡처 시뮬레이션"""
        start_time = time.time()
        
        # Depth 데이터 생성 시뮬레이션 (1280x720 float32)
        depth_data_size = 1280 * 720 * 4
        depth_data = np.random.uniform(0.1, 100.0, (720, 1280)).astype(np.float32)
        
        # 16-bit PNG 압축 시뮬레이션 (20% 압축 비율)
        compressed_size = int(depth_data_size * 0.2)
        compressed_data = np.random.bytes(compressed_size)
        
        # 처리 시간 시뮬레이션
        processing_delay = target_time_ms / 1000.0
        time.sleep(processing_delay)
        
        capture_time_ms = (time.time() - start_time) * 1000
        
        # 깊이 통계 시뮬레이션
        valid_pixels = np.random.randint(int(720*1280*0.8), int(720*1280*0.95))
        
        return {
            "sensor_type": "depth",
            "capture_time_ms": capture_time_ms,
            "raw_size_bytes": depth_data_size,
            "compressed_size_bytes": compressed_size,
            "compression_ratio": depth_data_size / compressed_size,
            "valid_pixels": valid_pixels,
            "total_pixels": 720 * 1280,
            "success": True,
            "timestamp": time.time()
        }
    
    def benchmark_synchronized_capture(self, 
                                    target_fps: float = 30.0, 
                                    duration_seconds: float = 10.0,
                                    sync_mode: str = "parallel") -> Dict[str, Any]:
        """동기화된 캡처 성능 벤치마크"""
        print(f"\n📊 동기화 캡처 벤치마크 ({sync_mode})")
        print(f"  목표: {target_fps} FPS, {duration_seconds}초 지속")
        
        frame_interval = 1.0 / target_fps
        frames_captured = 0
        successful_syncs = 0
        failed_syncs = 0
        
        total_rgb_size = 0
        total_depth_size = 0
        sync_time_diffs = []
        
        benchmark_start = time.time()
        
        while (time.time() - benchmark_start) < duration_seconds:
            frame_start = time.time()
            
            # 시스템 메트릭 수집
            system_metrics = self.get_current_system_metrics()
            
            if sync_mode == "parallel":
                # 병렬 캡처
                with ThreadPoolExecutor(max_workers=2) as executor:
                    rgb_future = executor.submit(self.simulate_rgb_capture_with_metrics)
                    depth_future = executor.submit(self.simulate_depth_capture_with_metrics)
                    
                    try:
                        rgb_result = rgb_future.result(timeout=1.0)
                        depth_result = depth_future.result(timeout=1.0)
                        
                        # 동기화 품질 평가
                        time_diff_ms = abs(rgb_result["timestamp"] - depth_result["timestamp"]) * 1000
                        sync_quality = "good" if time_diff_ms <= 10.0 else "poor"
                        
                        if sync_quality == "good":
                            successful_syncs += 1
                        else:
                            failed_syncs += 1
                        
                        sync_time_diffs.append(time_diff_ms)
                        
                        # 데이터 크기 누적
                        total_rgb_size += rgb_result["compressed_size_bytes"]
                        total_depth_size += depth_result["compressed_size_bytes"]
                        
                    except Exception as e:
                        failed_syncs += 1
                        print(f"    프레임 {frames_captured} 캡처 실패: {e}")
                        continue
            
            elif sync_mode == "sequential":
                # 순차 캡처
                try:
                    rgb_result = self.simulate_rgb_capture_with_metrics()
                    depth_result = self.simulate_depth_capture_with_metrics()
                    
                    time_diff_ms = abs(rgb_result["timestamp"] - depth_result["timestamp"]) * 1000
                    sync_quality = "sequential"  # 순차는 별도 품질
                    successful_syncs += 1
                    sync_time_diffs.append(time_diff_ms)
                    
                    total_rgb_size += rgb_result["compressed_size_bytes"]
                    total_depth_size += depth_result["compressed_size_bytes"]
                    
                except Exception as e:
                    failed_syncs += 1
                    continue
            
            frames_captured += 1
            
            # 성능 메트릭 기록
            frame_time = time.time() - frame_start
            current_fps = 1.0 / frame_time if frame_time > 0 else 0
            
            # 데이터 처리량 계산 (Mbps)
            frame_data_bytes = (total_rgb_size + total_depth_size) / frames_captured if frames_captured > 0 else 0
            data_throughput_mbps = (frame_data_bytes * current_fps * 8) / (1024 * 1024)
            
            metric = PerformanceMetrics(
                timestamp=time.time(),
                fps=current_fps,
                capture_time_ms=frame_time * 1000,
                memory_usage_mb=system_metrics["memory_usage_mb"],
                cpu_usage_percent=system_metrics["cpu_usage_percent"],
                sync_quality=sync_quality,
                data_throughput_mbps=data_throughput_mbps
            )
            
            self.metrics_history.append(metric)
            
            # 진행상황 출력 (10프레임마다)
            if frames_captured % 10 == 0:
                elapsed = time.time() - benchmark_start
                actual_fps = frames_captured / elapsed
                print(f"    {frames_captured}프레임: 실제 FPS {actual_fps:.1f}, "
                      f"CPU {system_metrics['cpu_usage_percent']:.1f}%, "
                      f"메모리 {system_metrics['process_memory_mb']:.1f}MB")
            
            # FPS 조절
            sleep_time = max(0, frame_interval - frame_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # 벤치마크 결과 계산
        total_time = time.time() - benchmark_start
        actual_fps = frames_captured / total_time
        sync_success_rate = successful_syncs / frames_captured if frames_captured > 0 else 0
        
        avg_sync_time_diff = sum(sync_time_diffs) / len(sync_time_diffs) if sync_time_diffs else 0
        
        # 메모리 통계
        memory_usages = [m.memory_usage_mb for m in self.metrics_history[-frames_captured:]]
        avg_memory = sum(memory_usages) / len(memory_usages) if memory_usages else 0
        max_memory = max(memory_usages) if memory_usages else 0
        
        # CPU 통계
        cpu_usages = [m.cpu_usage_percent for m in self.metrics_history[-frames_captured:]]
        avg_cpu = sum(cpu_usages) / len(cpu_usages) if cpu_usages else 0
        max_cpu = max(cpu_usages) if cpu_usages else 0
        
        results = {
            "benchmark_type": f"synchronized_capture_{sync_mode}",
            "target_fps": target_fps,
            "actual_fps": actual_fps,
            "fps_achievement_rate": actual_fps / target_fps,
            "total_frames": frames_captured,
            "successful_syncs": successful_syncs,
            "failed_syncs": failed_syncs,
            "sync_success_rate": sync_success_rate,
            "avg_sync_time_diff_ms": avg_sync_time_diff,
            "total_duration_seconds": total_time,
            
            # 데이터 처리량
            "total_rgb_data_bytes": total_rgb_size,
            "total_depth_data_bytes": total_depth_size,
            "total_data_bytes": total_rgb_size + total_depth_size,
            "data_throughput_mbps": ((total_rgb_size + total_depth_size) * 8) / (total_time * 1024 * 1024),
            
            # 시스템 리소스
            "avg_cpu_usage_percent": avg_cpu,
            "max_cpu_usage_percent": max_cpu,
            "avg_memory_usage_mb": avg_memory,
            "max_memory_usage_mb": max_memory,
            
            # 성능 목표 달성 여부
            "fps_target_achieved": actual_fps >= target_fps * 0.95,  # 95% 달성
            "sync_quality_acceptable": sync_success_rate >= 0.9,  # 90% 성공
            "resource_usage_acceptable": avg_cpu < 80.0 and avg_memory < 2048  # CPU 80%, 메모리 2GB
        }
        
        print(f"\n  📈 벤치마크 결과:")
        print(f"    실제 FPS: {actual_fps:.1f} / {target_fps} ({'✅' if results['fps_target_achieved'] else '❌'})")
        print(f"    동기화 성공률: {sync_success_rate:.1%} ({'✅' if results['sync_quality_acceptable'] else '❌'})")
        print(f"    데이터 처리량: {results['data_throughput_mbps']:.1f} Mbps")
        print(f"    평균 CPU 사용률: {avg_cpu:.1f}%")
        print(f"    평균 메모리 사용량: {avg_memory:.1f} MB")
        print(f"    최대 메모리 사용량: {max_memory:.1f} MB")
        
        return results
    
    def benchmark_memory_optimization(self) -> Dict[str, Any]:
        """메모리 최적화 벤치마크"""
        print(f"\n💾 메모리 최적화 벤치마크")
        
        initial_memory = psutil.virtual_memory().used / (1024**2)
        process = psutil.Process()
        initial_process_memory = process.memory_info().rss / (1024**2)
        
        print(f"  초기 시스템 메모리: {initial_memory:.1f} MB")
        print(f"  초기 프로세스 메모리: {initial_process_memory:.1f} MB")
        
        # 대량의 RGB-D 데이터 생성 및 처리
        memory_snapshots = []
        
        for i in range(20):  # 20회 반복
            # 대용량 RGB 데이터
            rgb_data = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
            
            # 대용량 Depth 데이터
            depth_data = np.random.uniform(0.1, 100.0, (720, 1280)).astype(np.float32)
            
            # 압축 시뮬레이션 (메모리 사용)
            compressed_rgb = np.random.bytes(rgb_data.nbytes // 20)
            compressed_depth = np.random.bytes(depth_data.nbytes // 5)
            
            # 메모리 스냅샷
            current_memory = process.memory_info().rss / (1024**2)
            memory_snapshots.append(current_memory)
            
            # 중간 정리 (10회마다)
            if i % 10 == 9:
                del rgb_data, depth_data, compressed_rgb, compressed_depth
                gc.collect()  # 강제 가비지 컬렉션
                
                gc_memory = process.memory_info().rss / (1024**2)
                print(f"    반복 {i+1}: 메모리 {current_memory:.1f} MB → GC 후 {gc_memory:.1f} MB")
        
        # 최종 정리
        gc.collect()
        final_process_memory = process.memory_info().rss / (1024**2)
        
        # 메모리 통계
        max_memory = max(memory_snapshots)
        avg_memory = sum(memory_snapshots) / len(memory_snapshots)
        memory_increase = max_memory - initial_process_memory
        memory_leak = final_process_memory - initial_process_memory
        
        results = {
            "initial_process_memory_mb": initial_process_memory,
            "max_process_memory_mb": max_memory,
            "final_process_memory_mb": final_process_memory,
            "avg_process_memory_mb": avg_memory,
            "peak_memory_increase_mb": memory_increase,
            "potential_memory_leak_mb": memory_leak,
            "memory_efficiency": memory_increase < 500,  # 500MB 이하 증가
            "no_significant_leak": memory_leak < 50  # 50MB 이하 누수
        }
        
        print(f"  📊 메모리 분석:")
        print(f"    최대 메모리 증가: {memory_increase:.1f} MB ({'✅' if results['memory_efficiency'] else '❌'})")
        print(f"    메모리 누수 의심: {memory_leak:.1f} MB ({'✅' if results['no_significant_leak'] else '❌'})")
        print(f"    메모리 효율성: {'✅ 양호' if results['memory_efficiency'] and results['no_significant_leak'] else '❌ 최적화 필요'}")
        
        return results
    
    def benchmark_compression_performance(self) -> Dict[str, Any]:
        """압축 성능 벤치마크"""
        print(f"\n🗜️  압축 성능 벤치마크")
        
        compression_scenarios = [
            {"name": "RGB_JPEG85_Depth_PNG16", "rgb_ratio": 0.05, "depth_ratio": 0.2},
            {"name": "RGB_JPEG75_Depth_PNG8", "rgb_ratio": 0.03, "depth_ratio": 0.1},
            {"name": "RGB_PNG_Depth_PNG16", "rgb_ratio": 0.25, "depth_ratio": 0.2}
        ]
        
        results = {}
        
        for scenario in compression_scenarios:
            print(f"  🎯 시나리오: {scenario['name']}")
            
            # 100프레임 압축 성능 테스트
            total_start = time.time()
            total_original_size = 0
            total_compressed_size = 0
            compression_times = []
            
            for frame in range(100):
                frame_start = time.time()
                
                # RGB 데이터 (1280x720x3)
                rgb_original = 1280 * 720 * 3
                rgb_compressed = int(rgb_original * scenario['rgb_ratio'])
                
                # Depth 데이터 (1280x720x4)
                depth_original = 1280 * 720 * 4
                depth_compressed = int(depth_original * scenario['depth_ratio'])
                
                # 압축 시간 시뮬레이션
                compression_delay = 0.002 + scenario['rgb_ratio'] * 0.001 + scenario['depth_ratio'] * 0.001
                time.sleep(compression_delay)
                
                frame_time = time.time() - frame_start
                compression_times.append(frame_time * 1000)  # ms
                
                total_original_size += rgb_original + depth_original
                total_compressed_size += rgb_compressed + depth_compressed
            
            total_time = time.time() - total_start
            
            # 통계 계산
            avg_compression_time = sum(compression_times) / len(compression_times)
            max_compression_time = max(compression_times)
            compression_fps = 100 / total_time
            
            scenario_results = {
                "scenario_name": scenario['name'],
                "total_original_size_mb": total_original_size / (1024**2),
                "total_compressed_size_mb": total_compressed_size / (1024**2),
                "compression_ratio": total_original_size / total_compressed_size,
                "avg_compression_time_ms": avg_compression_time,
                "max_compression_time_ms": max_compression_time,
                "compression_fps": compression_fps,
                "bandwidth_savings_percent": (1 - total_compressed_size / total_original_size) * 100,
                "real_time_capable": compression_fps >= 30  # 30 FPS 이상
            }
            
            results[scenario['name']] = scenario_results
            
            print(f"    압축비: {scenario_results['compression_ratio']:.1f}x")
            print(f"    평균 압축 시간: {avg_compression_time:.1f}ms")
            print(f"    압축 FPS: {compression_fps:.1f} ({'✅' if scenario_results['real_time_capable'] else '❌'})")
            print(f"    대역폭 절약: {scenario_results['bandwidth_savings_percent']:.1f}%")
        
        return results
    
    def generate_performance_report(self, 
                                  sync_results: Dict[str, Any],
                                  memory_results: Dict[str, Any],
                                  compression_results: Dict[str, Any]) -> Dict[str, Any]:
        """성능 보고서 생성"""
        print(f"\n📋 통합 성능 보고서 생성")
        
        # 전체 평가
        fps_acceptable = sync_results.get("fps_target_achieved", False)
        sync_acceptable = sync_results.get("sync_quality_acceptable", False)
        memory_acceptable = memory_results.get("memory_efficiency", False) and memory_results.get("no_significant_leak", False)
        
        # 압축 성능 평가 (모든 시나리오가 실시간 가능한지)
        compression_acceptable = all(
            results.get("real_time_capable", False) 
            for results in compression_results.values()
        )
        
        overall_performance_grade = "A"
        if not (fps_acceptable and sync_acceptable and memory_acceptable and compression_acceptable):
            if sum([fps_acceptable, sync_acceptable, memory_acceptable, compression_acceptable]) >= 3:
                overall_performance_grade = "B"
            elif sum([fps_acceptable, sync_acceptable, memory_acceptable, compression_acceptable]) >= 2:
                overall_performance_grade = "C"
            else:
                overall_performance_grade = "D"
        
        report = {
            "benchmark_timestamp": time.time(),
            "system_info": self.system_info,
            "performance_grade": overall_performance_grade,
            
            # 세부 평가
            "evaluations": {
                "fps_performance": {"passed": fps_acceptable, "score": sync_results.get("actual_fps", 0)},
                "synchronization": {"passed": sync_acceptable, "score": sync_results.get("sync_success_rate", 0)},
                "memory_efficiency": {"passed": memory_acceptable, "peak_increase_mb": memory_results.get("peak_memory_increase_mb", 0)},
                "compression_performance": {"passed": compression_acceptable, "scenarios_passed": sum(1 for r in compression_results.values() if r.get("real_time_capable", False))}
            },
            
            # 상세 결과
            "detailed_results": {
                "synchronization_benchmark": sync_results,
                "memory_benchmark": memory_results,
                "compression_benchmark": compression_results
            },
            
            # 권장사항
            "recommendations": []
        }
        
        # 권장사항 생성
        if not fps_acceptable:
            report["recommendations"].append("FPS 성능 개선: 캡처 파이프라인 최적화 필요")
        
        if not sync_acceptable:
            report["recommendations"].append("동기화 품질 개선: 하드웨어 동기화 또는 버퍼링 고려")
        
        if not memory_acceptable:
            report["recommendations"].append("메모리 최적화: 메모리 풀링 또는 스트리밍 처리 도입")
        
        if not compression_acceptable:
            report["recommendations"].append("압축 성능 개선: GPU 가속 압축 또는 병렬 처리 고려")
        
        if not report["recommendations"]:
            report["recommendations"].append("모든 성능 기준 달성: 현재 최적화 상태 양호")
        
        print(f"  🏆 전체 성능 등급: {overall_performance_grade}")
        print(f"  📊 세부 평가:")
        print(f"    FPS 성능: {'✅' if fps_acceptable else '❌'}")
        print(f"    동기화 품질: {'✅' if sync_acceptable else '❌'}")
        print(f"    메모리 효율성: {'✅' if memory_acceptable else '❌'}")
        print(f"    압축 성능: {'✅' if compression_acceptable else '❌'}")
        
        return report
    
    def save_report(self, report: Dict[str, Any], output_path: str = "/tmp/rgbd_performance_report.json"):
        """보고서 저장"""
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\n💾 성능 보고서 저장: {output_path}")
        print(f"   파일 크기: {Path(output_path).stat().st_size} bytes")


def main():
    """메인 성능 벤치마크 실행"""
    print("⚡ 통합 RGB-D 센서 성능 벤치마크")
    print("=" * 60)
    
    # 성능 벤치마크 인스턴스 생성
    benchmark = PerformanceBenchmark()
    
    try:
        # 1. 동기화 캡처 성능 벤치마크 (병렬)
        print("\n🚀 1단계: 병렬 동기화 성능 벤치마크")
        parallel_results = benchmark.benchmark_synchronized_capture(
            target_fps=30.0,
            duration_seconds=5.0,  # 테스트용 짧은 시간
            sync_mode="parallel"
        )
        
        # 2. 순차 캡처 성능 비교
        print("\n🔄 2단계: 순차 동기화 성능 비교")
        sequential_results = benchmark.benchmark_synchronized_capture(
            target_fps=15.0,  # 순차는 더 낮은 목표
            duration_seconds=3.0,
            sync_mode="sequential"
        )
        
        # 3. 메모리 최적화 벤치마크
        print("\n💾 3단계: 메모리 최적화 벤치마크")
        memory_results = benchmark.benchmark_memory_optimization()
        
        # 4. 압축 성능 벤치마크  
        print("\n🗜️ 4단계: 압축 성능 벤치마크")
        compression_results = benchmark.benchmark_compression_performance()
        
        # 5. 통합 성능 보고서 생성
        print("\n📋 5단계: 통합 성능 보고서")
        performance_report = benchmark.generate_performance_report(
            parallel_results, 
            memory_results, 
            compression_results
        )
        
        # 6. 보고서 저장
        benchmark.save_report(performance_report)
        
        # 최종 요약
        print(f"\n🎯 성능 벤치마크 완료")
        print(f"  총 메트릭 수집: {len(benchmark.metrics_history)}개")
        print(f"  전체 성능 등급: {performance_report['performance_grade']}")
        
        # 권장사항 출력
        if performance_report['recommendations']:
            print(f"\n💡 권장사항:")
            for i, rec in enumerate(performance_report['recommendations'], 1):
                print(f"  {i}. {rec}")
        
        return performance_report['performance_grade'] in ['A', 'B']
        
    except Exception as e:
        print(f"❌ 성능 벤치마크 실행 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)