#!/usr/bin/env python3
"""
RGB 카메라 센서 성능 비교 테스트
기본 버전 vs 개선 버전 성능 및 품질 비교
"""

import sys
import time
from pathlib import Path
import json

def run_rgb_sensor_comparison():
    """RGB 카메라 센서 비교 테스트"""
    print("🔬 RGB 카메라 센서 성능 비교 테스트")
    print("=" * 60)
    
    try:
        # Isaac Sim 초기화
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
        
        print("✅ Isaac Sim 환경 준비 완료")
        
        # 테스트 환경 설정 (기본)
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        usd_context = get_context()
        stage = usd_context.get_stage()
        
        if stage is None:
            stage = Usd.Stage.CreateNew("comparison_stage.usd")
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
        dome_light.CreateIntensityAttr(2000.0)
        dome_light.CreateColorAttr(Gf.Vec3f(0.9, 0.95, 1.0))
        
        # 테스트 객체들
        ground_geom = UsdGeom.Cube.Define(stage, Sdf.Path("/World/GroundPlane"))
        ground_geom.CreateSizeAttr(1.0)
        ground_geom.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))
        ground_geom.AddScaleOp().Set(Gf.Vec3f(20, 20, 1))
        
        # 다채로운 객체들
        colors = [
            (Gf.Vec3f(1, 0, 0), "RedCube", Gf.Vec3f(-2, 0, 1)),
            (Gf.Vec3f(0, 1, 0), "GreenSphere", Gf.Vec3f(0, 0, 1.5)),
            (Gf.Vec3f(0, 0, 1), "BlueCylinder", Gf.Vec3f(2, 0, 1))
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
        
        # 카메라 마운트
        sensors_group = stage.DefinePrim(Sdf.Path("/World/Sensors"), "Xform")
        cam_mount = stage.DefinePrim(Sdf.Path("/World/Sensors/FrontCam_Mount"), "Xform")
        cam_mount_xform = UsdGeom.XformCommonAPI(UsdGeom.Xformable(cam_mount))
        cam_mount_xform.SetTranslate(Gf.Vec3d(-8.0, 0.0, 3.0))
        cam_mount_xform.SetRotate(Gf.Vec3f(0, 10, 0))
        
        # 카메라 (개선된 설정)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Sensors/FrontCam_Mount/FrontCam"))
        camera_prim.CreateHorizontalApertureAttr(36.0)
        camera_prim.CreateVerticalApertureAttr(20.25)
        camera_prim.CreateFocalLengthAttr(24.0)
        camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
        
        # 시뮬레이션 준비
        import omni.kit.app
        app = omni.kit.app.get_app()
        for i in range(15):
            app.update()
        
        print("✅ 테스트 환경 준비 완료")
        
        # 기본 RGB 센서 테스트
        print("\n📷 기본 RGB 센서 테스트...")
        try:
            # 동적 import 방식
            import importlib.util
            
            spec = importlib.util.spec_from_file_location("rgb_camera_sensor", 
                "/home/roarm_m3/dev_roarm/roarm_mcp/sensors/rgb_camera_sensor.py")
            basic_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(basic_module)
            
            basic_sensor = basic_module.RGBCameraSensor(
                camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
                resolution=(1280, 720),
                output_dir="/tmp/basic_rgb_test"
            )
            
            if basic_sensor.initialize():
                print("  ✅ 기본 센서 초기화 성공")
                
                # 기본 센서 성능 측정
                basic_results = []
                for i in range(3):
                    result = basic_sensor.capture_and_encode_image(
                        format="JPEG", quality=85, base64_encode=False
                    )
                    if result:
                        basic_results.append(result)
                        print(f"    시도 {i+1}: {result['size_bytes']} bytes, "
                              f"{result['capture_time_ms']:.1f}ms")
                    time.sleep(0.1)
                
                basic_sensor.cleanup()
            else:
                print("  ❌ 기본 센서 초기화 실패")
                basic_results = []
                
        except Exception as e:
            print(f"  ❌ 기본 센서 테스트 오류: {e}")
            basic_results = []
        
        # 개선된 RGB 센서 테스트
        print("\n🎯 개선된 RGB 센서 테스트...")
        try:
            spec = importlib.util.spec_from_file_location("enhanced_rgb_camera_sensor", 
                "/home/roarm_m3/dev_roarm/roarm_mcp/sensors/enhanced_rgb_camera_sensor.py")
            enhanced_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(enhanced_module)
            
            enhanced_sensor = enhanced_module.EnhancedRGBCameraSensor(
                camera_path="/World/Sensors/FrontCam_Mount/FrontCam",
                resolution=(1280, 720),
                output_dir="/tmp/enhanced_rgb_test",
                fps=30.0
            )
            
            if enhanced_sensor.initialize():
                print("  ✅ 개선 센서 초기화 성공")
                
                # 개선 센서 성능 측정 (두 방법 모두)
                enhanced_results = {"direct": [], "file": []}
                
                for method in ["direct", "file"]:
                    print(f"    {method} 방법 테스트...")
                    for i in range(3):
                        result = enhanced_sensor.capture_and_encode_image(
                            format="JPEG", quality=85, base64_encode=False, method=method
                        )
                        if result:
                            enhanced_results[method].append(result)
                            print(f"      시도 {i+1}: {result['size_bytes']} bytes, "
                                  f"{result['capture_time_ms']:.1f}ms")
                        time.sleep(0.1)
                
                enhanced_sensor.cleanup()
            else:
                print("  ❌ 개선 센서 초기화 실패")
                enhanced_results = {"direct": [], "file": []}
                
        except Exception as e:
            print(f"  ❌ 개선 센서 테스트 오류: {e}")
            enhanced_results = {"direct": [], "file": []}
        
        # 결과 분석 및 비교
        print("\n📊 성능 비교 결과")
        print("=" * 60)
        
        def analyze_results(results, name):
            if not results:
                print(f"{name}: 데이터 없음")
                return None
                
            sizes = [r['size_bytes'] for r in results]
            times = [r['capture_time_ms'] for r in results]
            
            avg_size = sum(sizes) / len(sizes)
            avg_time = sum(times) / len(times)
            fps = 1000.0 / avg_time if avg_time > 0 else 0
            
            print(f"{name}:")
            print(f"  평균 파일 크기: {avg_size:,.0f} bytes")
            print(f"  평균 캡처 시간: {avg_time:.1f}ms")
            print(f"  예상 FPS: {fps:.1f}")
            print(f"  테스트 횟수: {len(results)}")
            
            return {
                "avg_size": avg_size,
                "avg_time": avg_time,
                "fps": fps,
                "count": len(results)
            }
        
        basic_stats = analyze_results(basic_results, "기본 RGB 센서")
        enhanced_direct_stats = analyze_results(enhanced_results["direct"], "개선 RGB 센서 (Direct)")
        enhanced_file_stats = analyze_results(enhanced_results["file"], "개선 RGB 센서 (File)")
        
        # 비교 요약
        print("\n🏆 비교 요약:")
        if basic_stats and enhanced_direct_stats:
            time_improvement = basic_stats["avg_time"] / enhanced_direct_stats["avg_time"] * 100 - 100
            fps_improvement = enhanced_direct_stats["fps"] / basic_stats["fps"] * 100 - 100
            
            print(f"  Direct 방법 성능 개선:")
            print(f"    캡처 시간: {time_improvement:+.1f}% 변화")
            print(f"    FPS: {fps_improvement:+.1f}% 향상")
        
        if enhanced_direct_stats and enhanced_file_stats:
            direct_vs_file = enhanced_direct_stats["avg_time"] / enhanced_file_stats["avg_time"] * 100 - 100
            print(f"  Direct vs File 방법:")
            print(f"    Direct가 {abs(direct_vs_file):.1f}% {'더 빠름' if direct_vs_file < 0 else '더 느림'}")
        
        # 추가 분석
        print("\n🔍 카메라 파라미터 분석:")
        if enhanced_results["direct"]:
            sample = enhanced_results["direct"][0]
            if 'camera_info' in sample:
                info = sample['camera_info']
                print(f"  초점거리: fx={info['fx']:.1f}, fy={info['fy']:.1f}")
                print(f"  주점: cx={info['cx']:.1f}, cy={info['cy']:.1f}")
                print(f"  해상도: {sample['resolution']}")
        
        # 생성된 파일 분석
        def check_output_files(directory, sensor_name):
            path = Path(directory)
            if path.exists():
                files = list(path.glob("*"))
                total_size = sum(f.stat().st_size for f in files if f.is_file())
                print(f"  {sensor_name}: {len(files)}개 파일, 총 {total_size:,} bytes")
                
                # camera_info.json 확인
                camera_info_file = path / "camera_info.json"
                if camera_info_file.exists():
                    print(f"    ✅ camera_info.json 생성됨")
                else:
                    print(f"    ❌ camera_info.json 없음")
            else:
                print(f"  {sensor_name}: 출력 디렉토리 없음")
        
        print("\n📁 생성된 파일 분석:")
        check_output_files("/tmp/basic_rgb_test", "기본 센서")
        check_output_files("/tmp/enhanced_rgb_test", "개선 센서")
        
        simulation_app.close()
        print("\n✅ RGB 센서 비교 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ 비교 테스트 오류: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = run_rgb_sensor_comparison()
    sys.exit(0 if success else 1)