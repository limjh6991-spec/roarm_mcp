#!/usr/bin/env python3
"""Enhanced Depth Camera Sensor using Isaac Sim's Annotator.

This module provides a sophisticated depth camera sensor implementation that
leverages the Omni Replicator framework's `distance_to_camera` annotator.
This approach allows for direct capture of high-precision depth data from the
simulation, which is generally more efficient than file-based methods.

The module offers functionalities for:
- Capturing raw float32 depth maps in meters.
- Converting depth maps to 16-bit integer format (e.g., for PNG).
- Normalizing depth maps to 8-bit format (for visualization or JPEG).
- Compressing and encoding depth data for network transmission.
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
import traceback

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EnhancedDepthCameraSensor:
    """An enhanced depth camera sensor using the Replicator Annotator.

    This class provides a high-performance interface to the depth sensor by
    capturing data directly from the `distance_to_camera` annotator. It also
    includes methods for data validation, conversion, and compression.

    Attributes:
        camera_path (str): The prim path of the camera in the USD stage.
        resolution (Tuple[int, int]): The (width, height) resolution.
        output_dir (str): Directory for temporary files and logs.
        fps (float): The target capture frequency.
        depth_range (Tuple[float, float]): The min/max depth range in meters.
        rep: The Omni Replicator core module.
        render_product: The Replicator render product for the camera.
        depth_annotator: The Replicator annotator for capturing depth data.
        camera_matrix (np.ndarray): The calculated camera intrinsic matrix.
    """
    
    def __init__(self, 
                 camera_path: str = "/World/Camera",
                 resolution: Tuple[int, int] = (1280, 720),
                 output_dir: Optional[str] = None,
                 fps: float = 30.0,
                 depth_range: Tuple[float, float] = (0.1, 100.0)):
        """Initializes the EnhancedDepthCameraSensor.

        Args:
            camera_path (str): The prim path to the camera in the USD stage.
            resolution (Tuple[int, int]): The desired output resolution.
            output_dir (Optional[str]): Directory for logs and temp files.
            fps (float): The target capture frequency (FPS).
            depth_range (Tuple[float, float]): The valid depth range (min, max)
                in meters.
        """
        self.camera_path = camera_path
        self.resolution = resolution
        self.output_dir = output_dir or "/tmp/enhanced_depth_camera"
        self.fps = fps
        self.depth_range = depth_range
        
        # Isaac Sim modules (imported at runtime)
        self.rep = None
        self.render_product = None
        self.writer = None
        self.depth_annotator = None
        
        # Performance metrics
        self.capture_count = 0
        self.last_capture_time = 0.0
        
        self._calculate_camera_parameters()
        
        logger.info(f"EnhancedDepthCameraSensor created for path: {camera_path}")
        logger.info(f"  - Depth range: {depth_range[0]:.1f}m to {depth_range[1]:.1f}m")
    
    def _calculate_camera_parameters(self):
        """Calculates camera intrinsic parameters based on sensor settings."""
        width, height = self.resolution
        aspect_ratio = width / height
        
        self.horizontal_aperture = 36.0  # mm
        self.vertical_aperture = self.horizontal_aperture / aspect_ratio
        self.focal_length = 24.0  # mm
        
        self.fx = self.focal_length * width / self.horizontal_aperture
        self.fy = self.focal_length * height / self.vertical_aperture
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
        self.distortion_coeffs = np.zeros(5)
        
        logger.info("Camera intrinsic parameters calculated.")

    def initialize(self) -> bool:
        """Initializes the sensor within an active Isaac Sim environment.
        
        Sets up the Replicator render product and attaches the depth annotator.

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
            
            self.depth_annotator = self.rep.AnnotatorRegistry.get_annotator(
                "distance_to_camera"
            )
            self.depth_annotator.attach([self.render_product])
            
            self._save_camera_info()
            logger.info(f"Depth camera sensor initialized successfully for: {self.camera_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize depth camera sensor: {e}", exc_info=True)
            return False
    
    def _save_camera_info(self):
        """Saves the camera's intrinsic and extrinsic parameters to a JSON file."""
        info_path = Path(self.output_dir) / "depth_camera_info.json"
        with open(info_path, 'w') as f:
            json.dump(self.get_performance_metrics(), f, indent=2)
        logger.info(f"Depth camera info saved to: {info_path}")

    def _validate_depth_data(self, depth_data: np.ndarray) -> Tuple[bool, Dict[str, float]]:
        """Validates the captured depth data and computes statistics.

        Args:
            depth_data (np.ndarray): The raw depth data array.

        Returns:
            Tuple[bool, Dict[str, float]]: A tuple containing a validity flag
            and a dictionary of statistics.
        """
        if depth_data is None or depth_data.size == 0:
            return False, {"reason": "Data is empty or None."}
        
        valid_mask = (depth_data > 0) & np.isfinite(depth_data)
        valid_pixels = np.sum(valid_mask)
        valid_ratio = valid_pixels / depth_data.size if depth_data.size > 0 else 0
        
        stats = {
            "valid_ratio": valid_ratio,
            "min_depth": float(np.min(depth_data[valid_mask])) if valid_pixels > 0 else 0.0,
            "max_depth": float(np.max(depth_data[valid_mask])) if valid_pixels > 0 else 0.0,
            "mean_depth": float(np.mean(depth_data[valid_mask])) if valid_pixels > 0 else 0.0
        }
        
        is_valid = valid_ratio > 0.01 and stats["max_depth"] > stats["min_depth"]
        if not is_valid:
            logger.warning(f"Invalid depth data detected (valid ratio: {valid_ratio:.2f}).")
        
        return is_valid, stats
    
    def capture_depth_image(self) -> Optional[np.ndarray]:
        """Captures a depth image directly from the annotator.
        
        Returns:
            Optional[np.ndarray]: The depth image as a float32 array (H, W) in
            meters, or None on failure.
        """
        if not self.depth_annotator:
            logger.error("Depth annotator is not initialized.")
            return None
        
        try:
            start_time = time.time()
            
            self.rep.orchestrator.step()
            depth_data = self.depth_annotator.get_data()
            
            if depth_data is None:
                logger.warning("Failed to get data from depth annotator.")
                return None
            
            depth_array = depth_data.cpu().numpy() if hasattr(depth_data, 'cpu') else np.array(depth_data)
            
            if depth_array.ndim == 3:
                depth_array = depth_array.squeeze(-1)
            
            if depth_array.dtype != np.float32:
                depth_array = depth_array.astype(np.float32)
            
            is_valid, _ = self._validate_depth_data(depth_array)
            if not is_valid:
                logger.warning("Captured depth data is not valid.")
                return None

            self.capture_count += 1
            self.last_capture_time = time.time() - start_time
            
            logger.debug(f"Depth image captured: {depth_array.shape}, time: {self.last_capture_time:.3f}s")
            return depth_array
            
        except Exception as e:
            logger.error(f"Failed to capture depth image directly: {e}", exc_info=True)
            return None
    
    def normalize_depth_to_8bit(self, depth_array: np.ndarray) -> np.ndarray:
        """Normalizes a float32 depth map to an 8-bit integer array.

        This is useful for visualization or lossy compression (e.g., JPEG).
        
        Args:
            depth_array (np.ndarray): The source depth array (float32, meters).
            
        Returns:
            np.ndarray: The normalized 8-bit depth array (0-255).
        """
        min_depth, max_depth = self.depth_range
        clipped_depth = np.clip(depth_array, min_depth, max_depth)
        normalized = (clipped_depth - min_depth) / (max_depth - min_depth) * 255.0
        normalized[~np.isfinite(depth_array)] = 0
        return normalized.astype(np.uint8)
    
    def convert_depth_to_16bit(self, depth_array: np.ndarray, scale: float = 1000.0) -> np.ndarray:
        """Converts a float32 depth map to a 16-bit integer array.

        This is standard for lossless depth storage (e.g., PNG), typically
        representing depth in millimeters.
        
        Args:
            depth_array (np.ndarray): The source depth array (float32, meters).
            scale (float): The scaling factor to apply before converting to int.
                           Defaults to 1000.0 (meters to millimeters).
            
        Returns:
            np.ndarray: The 16-bit depth array (uint16).
        """
        depth_mm = depth_array * scale
        # Clip to the valid range of uint16
        clipped_depth = np.clip(depth_mm, 0, 65535)
        clipped_depth[~np.isfinite(depth_array)] = 0
        return clipped_depth.astype(np.uint16)
    
    def compress_depth_image(self, 
                            depth_array: np.ndarray,
                            img_format: str = "PNG16") -> Optional[bytes]:
        """Compresses a depth image into a specified format.
        
        Args:
            depth_array (np.ndarray): The source depth array (float32, meters).
            img_format (str): The target format. Supported: "PNG16" (16-bit PNG),
                "PNG8" (8-bit normalized PNG), "JPEG8" (8-bit normalized JPEG).
            
        Returns:
            Optional[bytes]: The compressed image as a byte string, or None on failure.
        """
        try:
            if img_format == "PNG16":
                depth_16bit = self.convert_depth_to_16bit(depth_array)
                success, buffer = cv2.imencode('.png', depth_16bit, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            elif img_format == "PNG8":
                depth_8bit = self.normalize_depth_to_8bit(depth_array)
                success, buffer = cv2.imencode('.png', depth_8bit, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            elif img_format == "JPEG8":
                depth_8bit = self.normalize_depth_to_8bit(depth_array)
                success, buffer = cv2.imencode('.jpg', depth_8bit, [cv2.IMWRITE_JPEG_QUALITY, 90])
            else:
                logger.error(f"Unsupported depth compression format: {img_format}")
                return None

            if not success:
                logger.error(f"Failed to encode depth image to {img_format}")
                return None
            
            return buffer.tobytes()
        except Exception as e:
            logger.error(f"Failed to compress depth image: {e}", exc_info=True)
            return None

    def capture_and_encode_depth(self,
                                 img_format: str = "PNG16",
                                 to_base64: bool = True) -> Optional[Dict[str, Any]]:
        """Captures, compresses, and optionally encodes a depth image.
        
        Args:
            img_format (str): The compression format (e.g., "PNG16").
            to_base64 (bool): If True, the output is a Base64 string.
            
        Returns:
            Optional[Dict[str, Any]]: A dictionary with image data and metadata.
        """
        try:
            depth_array = self.capture_depth_image()
            if depth_array is None:
                return None
            
            _, depth_stats = self._validate_depth_data(depth_array)
            
            compressed_bytes = self.compress_depth_image(depth_array, img_format)
            if compressed_bytes is None:
                return None
            
            result = {
                "timestamp": time.time(),
                "resolution": list(depth_array.shape[::-1]),
                "format": img_format.lower(),
                "size_bytes": len(compressed_bytes),
                "capture_time_ms": self.last_capture_time * 1000,
                "depth_stats": depth_stats,
            }
            
            if to_base64:
                result["depth_image_base64"] = base64.b64encode(compressed_bytes).decode('utf-8')
            else:
                result["depth_image_bytes"] = compressed_bytes
            
            logger.info(f"Depth image captured and encoded ({img_format}): {result['size_bytes']} bytes")
            return result
            
        except Exception as e:
            logger.error(f"Failed to capture and encode depth image: {e}", exc_info=True)
            return None
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Returns performance metrics and camera configuration.

        Returns:
            Dict[str, Any]: A dictionary of performance and configuration data.
        """
        return {
            "total_captures": self.capture_count,
            "last_capture_time_ms": self.last_capture_time * 1000,
            "camera_path": self.camera_path,
            "resolution": list(self.resolution),
            "depth_range_m": list(self.depth_range),
            "camera_intrinsics": {
                "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy,
            }
        }
    
    def cleanup(self):
        """Detaches the annotator to clean up resources."""
        try:
            if self.depth_annotator:
                self.depth_annotator.detach()
            logger.info("Enhanced depth camera sensor resources cleaned up.")
        except Exception as e:
            logger.error(f"Error during depth sensor cleanup: {e}", exc_info=True)


def test_enhanced_depth_camera_sensor():
    """A test function to demonstrate the EnhancedDepthCameraSensor."""
    print("🎯 Starting EnhancedDepthCameraSensor test.")
    
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
        
        # Add objects at various depths
        for i in range(5):
            dist = (i + 1) * 2.0
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(f"/World/Cube_{i}"))
            cube.AddTranslateOp().Set(Gf.Vec3f(i - 2, 0, dist))
        
        print("✅ USD test environment set up.")
        
        depth_sensor = EnhancedDepthCameraSensor(
            camera_path="/World/Camera",
            resolution=(640, 480),
            output_dir="/tmp/enhanced_depth_test",
        )
        
        if not depth_sensor.initialize():
            return False
        
        print("✅ Enhanced Depth Camera Sensor initialized.")
        
        import omni.kit.app
        for _ in range(5):
            omni.kit.app.get_app().update()
        
        print("\n📐 Testing depth image capture...")
        result = depth_sensor.capture_and_encode_depth(img_format="PNG16")
        if result:
            print(f"  ✅ PNG16 Capture successful: {result['size_bytes']} bytes.")
            stats = result['depth_stats']
            print(f"  📊 Depth Stats: valid_ratio={stats['valid_ratio']:.2f}, range=[{stats['min_depth']:.2f}, {stats['max_depth']:.2f}]")
        else:
            print("  ❌ PNG16 Capture failed.")

        result_jpeg = depth_sensor.capture_and_encode_depth(img_format="JPEG8")
        if result_jpeg:
            print(f"  ✅ JPEG8 Capture successful: {result_jpeg['size_bytes']} bytes.")
        else:
            print("  ❌ JPEG8 Capture failed.")

        print(f"\n📊 Performance Metrics: {json.dumps(depth_sensor.get_performance_metrics(), indent=2)}")
        
        depth_sensor.cleanup()
        simulation_app.close()
        
        print("\n✅ EnhancedDepthCameraSensor test completed successfully.")
        return True
        
    except Exception as e:
        print(f"❌ An error occurred during the test: {e}")
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_enhanced_depth_camera_sensor()
    sys.exit(0 if success else 1)