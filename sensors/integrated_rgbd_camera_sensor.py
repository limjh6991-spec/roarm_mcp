#!/usr/bin/env python3
"""Integrated RGB-D Camera Sensor System for Isaac Sim.

This module provides a high-level system for capturing synchronized RGB and
depth images from a single camera prim in Isaac Sim. It orchestrates the
`EnhancedRGBCameraSensor` and `EnhancedDepthCameraSensor` modules to provide
synchronized frames.
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
import importlib.util
import traceback

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Dynamically locate and import sensor modules
try:
    # Assuming the script is run from a location where 'sensors' is a package
    from .enhanced_rgb_camera_sensor import EnhancedRGBCameraSensor
    from .enhanced_depth_camera_sensor import EnhancedDepthCameraSensor
except ImportError:
    # Fallback for running as a standalone script
    logger.warning("Could not import sensors as a package. Using file-based import.")
    # This path might need adjustment depending on the execution context
    script_dir = Path(__file__).parent.resolve()

    rgb_spec = importlib.util.spec_from_file_location("enhanced_rgb_camera_sensor", script_dir / "enhanced_rgb_camera_sensor.py")
    rgb_module = importlib.util.module_from_spec(rgb_spec)
    rgb_spec.loader.exec_module(rgb_module)
    EnhancedRGBCameraSensor = rgb_module.EnhancedRGBCameraSensor

    depth_spec = importlib.util.spec_from_file_location("enhanced_depth_camera_sensor", script_dir / "enhanced_depth_camera_sensor.py")
    depth_module = importlib.util.module_from_spec(depth_spec)
    depth_spec.loader.exec_module(depth_module)
    EnhancedDepthCameraSensor = depth_module.EnhancedDepthCameraSensor


@dataclass
class CameraCaptureResult:
    """A dataclass to hold the result of a single sensor capture.

    Attributes:
        timestamp (float): The timestamp of the capture.
        sensor_type (str): The type of sensor ("rgb" or "depth").
        resolution (Tuple[int, int]): The image resolution (width, height).
        format (str): The compression format of the image.
        size_bytes (int): The size of the compressed image in bytes.
        capture_time_ms (float): The time taken for the capture operation.
        success (bool): True if the capture was successful.
        error_message (Optional[str]): An error message if capture failed.
        image_base64 (Optional[str]): The Base64 encoded image string.
    """
    timestamp: float
    sensor_type: str
    resolution: Tuple[int, int]
    format: str
    size_bytes: int
    capture_time_ms: float
    success: bool
    error_message: Optional[str] = None
    image_base64: Optional[str] = None


class IntegratedRGBDCameraSensor:
    """An integrated system for synchronized RGB and Depth camera capture.

    This class manages an RGB and a Depth sensor, capturing data from both
    as close in time as possible using a thread pool.

    Attributes:
        camera_path (str): The prim path to the camera in the USD stage.
        resolution (Tuple[int, int]): The capture resolution.
        fps (float): The target frames per second.
        synchronization_mode (str): The method for synchronization ("soft" or
            "sequential").
        rgb_sensor (Optional[EnhancedRGBCameraSensor]): The RGB sensor instance.
        depth_sensor (Optional[EnhancedDepthCameraSensor]): The depth sensor instance.
        executor (ThreadPoolExecutor): The thread pool for parallel captures.
    """
    
    def __init__(self,
                 camera_path: str = "/World/Camera",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0,
                 depth_range: Tuple[float, float] = (0.1, 100.0),
                 synchronization_mode: str = "soft"):
        """Initializes the IntegratedRGBDCameraSensor.
        
        Args:
            camera_path (str): The prim path to the camera in the USD stage.
            resolution (Tuple[int, int]): The desired image resolution.
            output_dir (Optional[str]): Directory for logs and temp files.
            fps (float): The target capture frequency (FPS).
            depth_range (Tuple[float, float]): The valid depth range in meters.
            synchronization_mode (str): The synchronization method. "soft" uses
                a thread pool for parallel capture. "sequential" captures
                RGB then Depth.
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/integrated_rgbd_camera"
        self.fps = fps
        self.depth_range = depth_range
        self.synchronization_mode = synchronization_mode
        
        self.rgb_sensor: Optional[EnhancedRGBCameraSensor] = None
        self.depth_sensor: Optional[EnhancedDepthCameraSensor] = None
        
        # Performance and sync metrics
        self.capture_count = 0
        self.last_sync_time = 0.0
        self.sync_errors = 0
        self.max_sync_tolerance_ms = 16.0 # Allow up to ~1 frame of diff at 60fps
        
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="RGBD_Capture")
        self.capture_lock = threading.Lock()
        
        logger.info(f"IntegratedRGBDCameraSensor created for path: {camera_path}")
    
    def initialize(self) -> bool:
        """Initializes the integrated sensor system.

        This method creates instances of the underlying RGB and Depth sensors
        and initializes them.
        
        Returns:
            bool: True if initialization is successful, False otherwise.
        """
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            
            if not self._initialize_sensors():
                raise RuntimeError("Failed to initialize underlying sensors.")
            
            self._save_integrated_camera_info()
            logger.info("Integrated RGB-D sensor initialized successfully.")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize Integrated RGB-D sensor: {e}", exc_info=True)
            return False
    
    def _initialize_sensors(self) -> bool:
        """Initializes the individual RGB and Depth sensors."""
        try:
            self.rgb_sensor = EnhancedRGBCameraSensor(
                camera_path=self.camera_path,
                resolution=self.resolution,
                output_dir=os.path.join(self.output_dir, "rgb"),
                fps=self.fps
            )
            
            self.depth_sensor = EnhancedDepthCameraSensor(
                camera_path=self.camera_path,
                resolution=self.resolution,
                output_dir=os.path.join(self.output_dir, "depth"),
                fps=self.fps,
                depth_range=self.depth_range
            )
            
            if not self.rgb_sensor.initialize() or not self.depth_sensor.initialize():
                return False
            
            logger.info("✅ Underlying RGB and Depth sensors initialized.")
            return True
            
        except Exception as e:
            logger.error(f"Error during sensor initialization: {e}", exc_info=True)
            return False
    
    def _save_integrated_camera_info(self):
        """Saves the combined configuration of the RGB-D sensor to a JSON file."""
        if not self.rgb_sensor or not self.depth_sensor:
            return
        
        info_path = Path(self.output_dir) / "integrated_camera_info.json"
        with open(info_path, 'w') as f:
            json.dump(self.get_performance_metrics(), f, indent=2)
        logger.info(f"Integrated camera info saved to: {info_path}")
    
    def _capture_rgb_task(self) -> CameraCaptureResult:
        """Task for capturing an RGB image asynchronously."""
        start_time = time.time()
        try:
            result = self.rgb_sensor.capture_and_encode(to_base64=True)
            if result:
                return CameraCaptureResult(
                    timestamp=result["timestamp"], sensor_type="rgb",
                    resolution=tuple(result["resolution"]), format=result["format"],
                    size_bytes=result["size_bytes"], capture_time_ms=result["capture_time_ms"],
                    success=True, image_base64=result.get("image_base64")
                )
        except Exception as e:
            logger.error(f"Exception in RGB capture task: {e}")

        return CameraCaptureResult(
            timestamp=time.time(), sensor_type="rgb", resolution=self.resolution,
            format="jpeg", size_bytes=0, capture_time_ms=(time.time() - start_time) * 1000,
            success=False, error_message="RGB capture failed"
        )
    
    def _capture_depth_task(self) -> CameraCaptureResult:
        """Task for capturing a depth image asynchronously."""
        start_time = time.time()
        try:
            result = self.depth_sensor.capture_and_encode_depth(to_base64=True)
            if result:
                return CameraCaptureResult(
                    timestamp=result["timestamp"], sensor_type="depth",
                    resolution=tuple(result["resolution"]), format=result["format"],
                    size_bytes=result["size_bytes"], capture_time_ms=result["capture_time_ms"],
                    success=True, image_base64=result.get("depth_image_base64")
                )
        except Exception as e:
            logger.error(f"Exception in depth capture task: {e}")

        return CameraCaptureResult(
            timestamp=time.time(), sensor_type="depth", resolution=self.resolution,
            format="png16", size_bytes=0, capture_time_ms=(time.time() - start_time) * 1000,
            success=False, error_message="Depth capture failed"
        )

    def capture_synchronized_rgbd(self) -> Dict[str, Any]:
        """Captures a synchronized pair of RGB and depth images.

        Depending on the `synchronization_mode`, this either captures in
        parallel using a thread pool or sequentially.
        
        Returns:
            Dict[str, Any]: A dictionary containing the synchronized RGB-D data
            and metadata.
        """
        with self.capture_lock:
            sync_start_time = time.time()
            try:
                if self.synchronization_mode == "sequential":
                    rgb_result = self._capture_rgb_task()
                    depth_result = self._capture_depth_task()
                else: # "soft" mode
                    rgb_future = self.executor.submit(self._capture_rgb_task)
                    depth_future = self.executor.submit(self._capture_depth_task)
                    rgb_result = rgb_future.result()
                    depth_result = depth_future.result()
                
                time_diff_ms = abs(rgb_result.timestamp - depth_result.timestamp) * 1000
                sync_quality = "good" if time_diff_ms <= self.max_sync_tolerance_ms else "poor"
                
                if sync_quality == "poor":
                    self.sync_errors += 1
                    logger.warning(f"Poor synchronization quality: {time_diff_ms:.1f}ms difference.")
                
                self.capture_count += 1
                self.last_sync_time = time.time() - sync_start_time
                
                return {
                    "sync_timestamp": (rgb_result.timestamp + depth_result.timestamp) / 2,
                    "sync_quality": sync_quality,
                    "time_difference_ms": time_diff_ms,
                    "overall_success": rgb_result.success and depth_result.success,
                    "rgb": asdict(rgb_result),
                    "depth": asdict(depth_result),
                }
                
            except Exception as e:
                logger.error(f"Failed to capture synchronized RGB-D frame: {e}", exc_info=True)
                return {"overall_success": False, "error_message": str(e)}
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Returns a dictionary of performance metrics for the integrated sensor."""
        rgb_metrics = self.rgb_sensor.get_performance_metrics() if self.rgb_sensor else {}
        depth_metrics = self.depth_sensor.get_performance_metrics() if self.depth_sensor else {}
        
        return {
            "integrated_metrics": {
                "total_sync_captures": self.capture_count,
                "last_sync_time_ms": self.last_sync_time * 1000,
                "sync_error_count": self.sync_errors,
                "sync_error_rate": self.sync_errors / self.capture_count if self.capture_count > 0 else 0,
            },
            "rgb_sensor_metrics": rgb_metrics,
            "depth_sensor_metrics": depth_metrics,
        }
    
    def cleanup(self):
        """Cleans up resources used by the sensor system."""
        try:
            if self.rgb_sensor:
                self.rgb_sensor.cleanup()
            if self.depth_sensor:
                self.depth_sensor.cleanup()
            self.executor.shutdown(wait=True)
            logger.info("Integrated RGB-D sensor resources cleaned up.")
        except Exception as e:
            logger.error(f"Error during resource cleanup: {e}", exc_info=True)


def test_integrated_rgbd_system():
    """A test function to demonstrate the IntegratedRGBDCameraSensor."""
    print("🎭 Starting Integrated RGB-D Camera System test.")
    
    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": True})
        
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        stage = Usd.Stage.CreateInMemory()
        get_context().set_stage(stage)
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Light")).CreateIntensityAttr(5000.0)
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Camera"))
        UsdGeom.Xformable(camera_prim).AddTranslateOp().Set(Gf.Vec3f(0, -5, 2))
        UsdGeom.Cube.Define(stage, Sdf.Path("/World/TestCube")).AddTranslateOp().Set(Gf.Vec3f(0, 0, 1))
        
        print("✅ USD test environment set up.")
        
        rgbd_sensor = IntegratedRGBDCameraSensor(
            camera_path="/World/Camera",
            resolution=(640, 480),
            synchronization_mode="soft"
        )
        
        if not rgbd_sensor.initialize():
            return False
        
        print("✅ Integrated RGB-D sensor initialized.")
        
        import omni.kit.app
        for _ in range(5):
            omni.kit.app.get_app().update()
        
        print("\n📷📐 Testing synchronized RGB-D capture...")
        result = rgbd_sensor.capture_synchronized_rgbd()
        
        if result["overall_success"]:
            print(f"  ✅ Sync successful (quality: {result['sync_quality']}).")
            print(f"  - RGB: {result['rgb']['size_bytes']} bytes, Depth: {result['depth']['size_bytes']} bytes.")
            print(f"  - Time difference: {result['time_difference_ms']:.2f} ms.")
        else:
            print(f"  ❌ Sync failed: {result.get('error_message', 'Unknown error')}")
            
        print(f"\n📊 Performance Metrics: {json.dumps(rgbd_sensor.get_performance_metrics(), indent=2)}")
        
        rgbd_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ Integrated RGB-D system test completed successfully.")
        return True
        
    except Exception as e:
        print(f"❌ An error occurred during the test: {e}")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_integrated_rgbd_system()
    sys.exit(0 if success else 1)