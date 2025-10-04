#!/usr/bin/env python3
"""Enhanced RGB Camera Sensor using Isaac Sim's Annotator.

This module provides a high-performance RGB camera sensor implementation using
the Omni Replicator framework's "rgb" annotator. This method allows for direct
memory capture of RGB data, which is significantly faster than writing files
to disk.

The module includes features for:
- Direct capture of RGB images from the simulation.
- Black-frame detection and retry mechanisms.
- Calculation of camera intrinsic parameters.
- High-quality image compression (JPEG, PNG) and Base64 encoding.
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
import traceback

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class EnhancedRGBCameraSensor:
    """An enhanced RGB camera sensor using the Replicator Annotator.

    This class provides a high-performance interface to an RGB camera by
    capturing data directly from the "rgb" annotator. It is optimized for
    real-time applications requiring low-latency image streaming.

    Attributes:
        camera_path (str): The prim path of the camera in the USD stage.
        resolution (Tuple[int, int]): The (width, height) resolution.
        output_dir (str): Directory for temporary files and logs.
        fps (float): The target capture frequency.
        rep: The Omni Replicator core module.
        render_product: The Replicator render product for the camera.
        annotator: The Replicator "rgb" annotator instance.
        camera_matrix (np.ndarray): The calculated camera intrinsic matrix.
    """
    
    def __init__(self, 
                 camera_path: str = "/World/Camera",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0):
        """Initializes the EnhancedRGBCameraSensor.

        Args:
            camera_path (str): The prim path to the camera in the USD stage.
            resolution (Tuple[int, int]): The desired output resolution.
            output_dir (Optional[str]): Directory for logs and temp files.
            fps (float): The target capture frequency (FPS).
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/enhanced_rgb_camera"
        self.fps = fps
        
        # Isaac Sim modules (imported at runtime)
        self.rep = None
        self.render_product = None
        self.annotator = None
        
        # Performance and validation
        self.capture_count = 0
        self.last_capture_time = 0.0
        self.frame_warmup_count = 3
        self.black_frame_threshold = 1.0
        
        # Compression settings
        self.jpeg_quality = 95
        self.png_compression = 3 # A good balance of speed and size
        
        self._calculate_camera_parameters()
        
        logger.info(f"EnhancedRGBCameraSensor created for path: {camera_path}")
    
    def _calculate_camera_parameters(self):
        """Calculates camera intrinsic parameters based on sensor settings."""
        width, height = self.resolution
        aspect_ratio = width / height
        
        self.horizontal_aperture = 36.0
        self.vertical_aperture = self.horizontal_aperture / aspect_ratio
        self.focal_length = 24.0
        
        self.fx = self.focal_length * width / self.horizontal_aperture
        self.fy = self.focal_length * height / self.vertical_aperture
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        self.camera_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        self.distortion_coeffs = np.zeros(5)
        
        logger.info("Camera intrinsic parameters calculated.")
    
    def initialize(self) -> bool:
        """Initializes the sensor in an active Isaac Sim environment.

        Sets up the Replicator render product and attaches the RGB annotator.
        
        Returns:
            bool: True if initialization is successful, False otherwise.
        """
        try:
            import omni.replicator.core as rep
            self.rep = rep
            
            os.makedirs(self.output_dir, exist_ok=True)
            
            self.render_product = self.rep.create.render_product(self.camera_path, self.resolution)
            self.annotator = self.rep.AnnotatorRegistry.get_annotator("rgb")
            self.annotator.attach([self.render_product])
            
            self._save_camera_info()
            logger.info(f"RGB camera sensor initialized successfully for: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize RGB camera sensor: {e}", exc_info=True)
            return False
    
    def _save_camera_info(self):
        """Saves the camera's configuration to a JSON file."""
        info_path = Path(self.output_dir) / "rgb_camera_info.json"
        with open(info_path, 'w') as f:
            json.dump(self.get_performance_metrics(), f, indent=2)
        logger.info(f"RGB camera info saved to: {info_path}")

    def _warmup_frames(self, count: int):
        """Runs a few simulation steps to prevent capturing black/unrendered frames.

        Args:
            count (int): The number of warmup frames to process.
        """
        try:
            import omni.kit.app
            logger.debug(f"Starting frame warmup ({count} frames)...")
            for i in range(count):
                omni.kit.app.get_app().update()
                if i == count - 1:
                    self.rep.orchestrator.step()
            logger.debug("Frame warmup complete.")
        except Exception as e:
            logger.warning(f"Error during frame warmup: {e}")
    
    def _is_black_frame(self, image: np.ndarray) -> bool:
        """Checks if the captured image is mostly black.

        Args:
            image (np.ndarray): The image to check.

        Returns:
            bool: True if the frame is considered black, False otherwise.
        """
        if image is None or image.size == 0:
            return True
        is_black = image.mean() < self.black_frame_threshold
        if is_black:
            logger.warning(f"Black frame detected (mean value: {image.mean():.2f}).")
        return is_black
    
    def capture_rgb_image(self) -> Optional[np.ndarray]:
        """Captures an RGB image directly from the annotator.
        
        Returns:
            Optional[np.ndarray]: The RGB image as a uint8 array (H, W, 3), or None on failure.
        """
        if not self.annotator:
            logger.error("RGB annotator is not initialized.")
            return None
        
        try:
            start_time = time.time()
            
            if self.capture_count == 0:
                self._warmup_frames(self.frame_warmup_count)
            
            self.rep.orchestrator.step()
            rgb_data = self.annotator.get_data()
            
            if rgb_data is None:
                logger.warning("Failed to get data from RGB annotator.")
                return None
            
            rgb_array = rgb_data.cpu().numpy() if hasattr(rgb_data, 'cpu') else np.array(rgb_data)
            
            if rgb_array.dtype != np.uint8:
                rgb_array = (rgb_array * 255).astype(np.uint8)
            
            if rgb_array.shape[-1] == 4:
                rgb_array = rgb_array[..., :3]
            
            if self._is_black_frame(rgb_array):
                logger.info("Retrying capture after black frame...")
                self._warmup_frames(1)
                # Recapture logic is handled by the calling function if needed
                return None

            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"RGB image captured: {rgb_array.shape}, time: {self.last_capture_time:.3f}s")
            return rgb_array
            
        except Exception as e:
            logger.error(f"Failed to capture RGB image directly: {e}", exc_info=True)
            return None
    
    def compress_image(self, image: np.ndarray, img_format: str = "JPEG", quality: Optional[int] = None) -> Optional[bytes]:
        """Compresses an RGB image into a specified format.
        
        Args:
            image (np.ndarray): The source RGB image array.
            img_format (str): Target format ("JPEG" or "PNG").
            quality (Optional[int]): JPEG quality (1-100) or PNG compression level (0-9).
            
        Returns:
            Optional[bytes]: The compressed image as a byte string, or None on failure.
        """
        try:
            bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            if img_format.upper() == "JPEG":
                params = [cv2.IMWRITE_JPEG_QUALITY, quality or self.jpeg_quality]
                ext = '.jpg'
            elif img_format.upper() == "PNG":
                params = [cv2.IMWRITE_PNG_COMPRESSION, quality or self.png_compression]
                ext = '.png'
            else:
                logger.error(f"Unsupported image format: {img_format}")
                return None
            
            success, encoded_img = cv2.imencode(ext, bgr_image, params)
            if not success:
                logger.error(f"Failed to encode image to {img_format}")
                return None
                
            return encoded_img.tobytes()
            
        except Exception as e:
            logger.error(f"Failed to compress image: {e}", exc_info=True)
            return None
    
    def capture_and_encode(self, img_format: str = "JPEG", quality: Optional[int] = None, to_base64: bool = True) -> Optional[Dict[str, Any]]:
        """Captures, compresses, and optionally encodes an image.
        
        Args:
            img_format (str): The compression format ("JPEG" or "PNG").
            quality (Optional[int]): Compression quality/level.
            to_base64 (bool): If True, output is Base64 string; otherwise, raw bytes.
            
        Returns:
            Optional[Dict[str, Any]]: A dictionary with image data and metadata.
        """
        try:
            rgb_image = self.capture_rgb_image()
            if rgb_image is None:
                # Retry once if it was a black frame
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
                "capture_time_ms": self.last_capture_time * 1000,
                "camera_info": {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy}
            }
            
            if to_base64:
                result["image_base64"] = base64.b64encode(compressed_bytes).decode('utf-8')
            else:
                result["image_bytes"] = compressed_bytes
            
            logger.info(f"Image captured and encoded ({img_format}): {result['size_bytes']} bytes")
            return result
            
        except Exception as e:
            logger.error(f"Failed to capture and encode image: {e}", exc_info=True)
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Returns performance metrics and camera configuration.

        Returns:
            Dict[str, Any]: A dictionary of performance and configuration data.
        """
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "target_fps": self.fps,
            "actual_fps": 1.0 / self.last_capture_time if self.last_capture_time > 0 else 0,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "camera_parameters": {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy}
        }
    
    def cleanup(self):
        """Detaches the annotator to clean up resources."""
        try:
            if self.annotator:
                self.annotator.detach()
            logger.info("Enhanced RGB camera sensor resources cleaned up.")
        except Exception as e:
            logger.error(f"Error during RGB sensor cleanup: {e}", exc_info=True)


def test_enhanced_rgb_camera_sensor():
    """A test function to demonstrate the EnhancedRGBCameraSensor."""
    print("🎥 Starting EnhancedRGBCameraSensor test.")
    
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
        
        camera_sensor = EnhancedRGBCameraSensor(
            camera_path="/World/Camera",
            resolution=(640, 480),
        )
        
        if not camera_sensor.initialize():
            return False
        
        print("✅ Enhanced RGB Camera Sensor initialized.")
        
        import omni.kit.app
        for _ in range(5):
            omni.kit.app.get_app().update()
        
        print("\n📷 Testing image capture...")
        result = camera_sensor.capture_and_encode(img_format="JPEG", quality=90)
        if result:
            print(f"  ✅ JPEG Capture successful: {result['size_bytes']} bytes.")
        else:
            print("  ❌ JPEG Capture failed.")

        result_png = camera_sensor.capture_and_encode(img_format="PNG")
        if result_png:
            print(f"  ✅ PNG Capture successful: {result_png['size_bytes']} bytes.")
        else:
            print("  ❌ PNG Capture failed.")

        print(f"\n📊 Performance Metrics: {json.dumps(camera_sensor.get_performance_metrics(), indent=2)}")
        
        camera_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ EnhancedRGBCameraSensor test completed successfully.")
        return True
        
    except Exception as e:
        print(f"❌ An error occurred during the test: {e}")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_enhanced_rgb_camera_sensor()
    sys.exit(0 if success else 1)