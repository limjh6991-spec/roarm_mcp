#!/usr/bin/env python3
"""
Replicator-based RGB Camera Sensor Module for Isaac Sim.

This module provides an alternative implementation of an RGB camera sensor using
the Omni Replicator framework. It captures images by writing them to disk and
then reading them back, which can be useful for certain debugging and data
generation workflows.

Note: For real-time applications, `sensors.camera_sensor.RGBCameraSensor` is
generally preferred due to its direct memory access, which offers better
performance.
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
import traceback

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RGBCameraSensor:
    """An RGB camera sensor using the Omni Replicator framework.

    This class initializes a camera, captures RGB images by writing them to
    a temporary directory, and provides methods for compression and encoding.

    Attributes:
        camera_path (str): The prim path of the camera in the USD stage.
        resolution (Tuple[int, int]): The (width, height) resolution for captures.
        output_dir (str): The directory where captured images are temporarily stored.
        rep: The Omni Replicator core module.
        render_product: The Replicator render product associated with the camera.
        writer: The Replicator writer for outputting image data.
        capture_count (int): A counter for the number of images captured.
        last_capture_time (float): The duration of the last capture operation.
        jpeg_quality (int): The default quality setting for JPEG compression.
    """
    
    def __init__(self, 
                 camera_path: str = "/World/Camera",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None):
        """Initializes the Replicator-based RGB Camera Sensor.
        
        Args:
            camera_path (str): The prim path to the camera in the USD stage.
            resolution (Tuple[int, int]): The desired output resolution (width, height).
            output_dir (Optional[str]): Directory to save captured images. If None,
                a default temporary directory is used.
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/replicator_rgb_camera"
        
        # Isaac Sim modules (imported at runtime)
        self.rep = None
        self.render_product = None
        self.writer = None
        
        # Performance metrics
        self.capture_count = 0
        self.last_capture_time = 0.0
        
        # Compression settings
        self.jpeg_quality = 85
        
        logger.info(f"RGBCameraSensor initialized for path: {self.camera_path}, resolution: {self.resolution}")
    
    def initialize(self) -> bool:
        """Initializes the sensor within an active Isaac Sim environment.

        This method imports Replicator, creates an output directory, and sets up
        a `render_product` and `BasicWriter` to capture RGB data from the camera.
        
        Returns:
            bool: True if initialization is successful, False otherwise.
        """
        try:
            import omni.replicator.core as rep
            self.rep = rep
            
            os.makedirs(self.output_dir, exist_ok=True)
            
            self.render_product = self.rep.create.render_product(
                self.camera_path, 
                self.resolution
            )
            
            self.writer = self.rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                output_dir=self.output_dir,
                rgb=True
            )
            self.writer.attach([self.render_product])
            
            logger.info(f"Camera sensor initialized successfully for: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize camera sensor: {e}", exc_info=True)
            return False
    
    def capture_rgb_image(self) -> Optional[np.ndarray]:
        """Captures a single RGB image.

        This method triggers a Replicator step, finds the most recently saved
        PNG file from the output directory, and loads it into a NumPy array.
        
        Returns:
            Optional[np.ndarray]: The captured RGB image as a NumPy array (H, W, 3),
            or None if capture fails.
        """
        if not self.rep or not self.render_product:
            logger.error("Camera sensor is not initialized.")
            return None
        
        try:
            start_time = time.time()
            
            self.rep.orchestrator.step()
            
            # Find the latest generated RGB file
            rgb_files = list(Path(self.output_dir).glob("rgb_*.png"))
            if not rgb_files:
                logger.warning("No RGB image file found in output directory.")
                return None
            
            latest_file = max(rgb_files, key=lambda x: x.stat().st_mtime)
            
            image = cv2.imread(str(latest_file))
            if image is None:
                logger.error(f"Failed to load image: {latest_file}")
                return None
            
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"RGB image captured: {rgb_image.shape}, time: {self.last_capture_time:.3f}s")
            return rgb_image
            
        except Exception as e:
            logger.error(f"Failed to capture RGB image: {e}", exc_info=True)
            return None
    
    def compress_image(self, 
                      image: np.ndarray, 
                      img_format: str = "JPEG",
                      quality: Optional[int] = None) -> Optional[bytes]:
        """Compresses an image into a specified format.
        
        Args:
            image (np.ndarray): The source RGB image array.
            img_format (str): The target compression format ("JPEG" or "PNG").
            quality (Optional[int]): The quality for JPEG compression (1-100).
                If None, uses the default `self.jpeg_quality`.
            
        Returns:
            Optional[bytes]: The compressed image as a byte string, or None on failure.
        """
        try:
            if quality is None:
                quality = self.jpeg_quality
                
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if img_format.upper() == "JPEG":
                params = [cv2.IMWRITE_JPEG_QUALITY, quality]
                ext = '.jpg'
            elif img_format.upper() == "PNG":
                params = [cv2.IMWRITE_PNG_COMPRESSION, 9]
                ext = '.png'
            else:
                logger.error(f"Unsupported image format: {img_format}")
                return None
            
            success, encoded_img = cv2.imencode(ext, bgr_image, params)
            if not success:
                logger.error("Failed to encode image.")
                return None
                
            return encoded_img.tobytes()
            
        except Exception as e:
            logger.error(f"Failed to compress image: {e}", exc_info=True)
            return None
    
    @staticmethod
    def encode_base64(image_bytes: bytes) -> str:
        """Encodes image bytes into a Base64 string.
        
        Args:
            image_bytes (bytes): The compressed image byte string.
            
        Returns:
            str: The Base64 encoded string.
        """
        return base64.b64encode(image_bytes).decode('utf-8')
    
    def capture_and_encode(self,
                           img_format: str = "JPEG",
                           quality: Optional[int] = None,
                           to_base64: bool = True) -> Optional[Dict[str, Any]]:
        """Captures, compresses, and optionally encodes an image in one call.
        
        Args:
            img_format (str): The compression format ("JPEG" or "PNG").
            quality (Optional[int]): JPEG quality (1-100).
            to_base64 (bool): If True, the output is a Base64 string. If False,
                it's raw bytes.
            
        Returns:
            Optional[Dict[str, Any]]: A dictionary containing image data and
            metadata, or None on failure.
        """
        try:
            rgb_image = self.capture_rgb_image()
            if rgb_image is None:
                return None
            
            compressed_bytes = self.compress_image(rgb_image, img_format, quality)
            if compressed_bytes is None:
                return None
            
            result = {
                "timestamp": time.time(),
                "resolution": list(rgb_image.shape[:2][::-1]),
                "format": img_format.lower(),
                "size_bytes": len(compressed_bytes),
                "capture_time_ms": self.last_capture_time * 1000
            }
            
            if to_base64:
                result["image_base64"] = self.encode_base64(compressed_bytes)
            else:
                result["image_bytes"] = compressed_bytes
            
            logger.info(f"Image captured: {result['format']}, {result['size_bytes']} bytes")
            return result
            
        except Exception as e:
            logger.error(f"Failed to capture and encode image: {e}", exc_info=True)
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Returns performance metrics for the sensor.
        
        Returns:
            Dict[str, Any]: A dictionary of performance data.
        """
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "output_directory": self.output_dir
        }
    
    def cleanup(self):
        """Detaches the Replicator writer to clean up resources."""
        try:
            if self.writer:
                self.writer.detach()
            logger.info("RGB camera sensor resources cleaned up.")
        except Exception as e:
            logger.error(f"Error during resource cleanup: {e}", exc_info=True)


def test_rgb_camera_sensor():
    """A test function to demonstrate the RGBCameraSensor functionality."""
    print("🎥 Starting RGBCameraSensor test.")
    
    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": True})
        print("✅ Isaac Sim application initialized.")
        
        from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf
        from omni.usd import get_context
        
        stage = Usd.Stage.CreateInMemory()
        get_context().set_stage(stage)
        
        world_prim = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_prim.GetPrim())
        
        UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight")).CreateIntensityAttr(5000.0)
        
        camera_prim = UsdGeom.Camera.Define(stage, Sdf.Path("/World/Camera"))
        xform = UsdGeom.Xformable(camera_prim)
        xform.AddTranslateOp().Set(Gf.Vec3f(-5.0, 0.0, 2.0))
        
        UsdGeom.Cube.Define(stage, Sdf.Path("/World/TestCube")).AddTranslateOp().Set(Gf.Vec3f(0, 0, 1))
        print("✅ USD environment set up.")
        
        camera_sensor = RGBCameraSensor(
            camera_path="/World/Camera",
            resolution=(640, 480),
            output_dir="/tmp/rgb_camera_test"
        )
        
        if not camera_sensor.initialize():
            print("❌ Failed to initialize camera sensor.")
            return False
        
        print("✅ RGB camera sensor initialized.")
        
        import omni.kit.app
        for _ in range(5):
            omni.kit.app.get_app().update()
        
        print("\n📷 Testing image capture...")
        for i in range(2):
            print(f"\n--- Capture {i+1}/2 ---")
            
            result_jpeg = camera_sensor.capture_and_encode(format="JPEG", quality=85)
            if result_jpeg:
                print(f"  JPEG: {result_jpeg['size_bytes']} bytes, {result_jpeg['capture_time_ms']:.1f}ms")
            
            result_png = camera_sensor.capture_and_encode(format="PNG", to_base64=False)
            if result_png:
                print(f"  PNG: {result_png['size_bytes']} bytes, {result_png['capture_time_ms']:.1f}ms")
            
            time.sleep(0.1)
        
        metrics = camera_sensor.get_performance_metrics()
        print(f"\n📊 Performance Metrics: {json.dumps(metrics, indent=2)}")
        
        camera_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ RGBCameraSensor test completed successfully.")
        return True
        
    except Exception as e:
        print(f"❌ An error occurred during the test: {e}")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_rgb_camera_sensor()
    sys.exit(0 if success else 1)