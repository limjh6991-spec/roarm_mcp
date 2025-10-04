#!/usr/bin/env python3
"""RGB Camera Sensor module for Isaac Sim.

This module provides a class to interface with an RGB camera within the NVIDIA
Isaac Sim environment. It handles the creation, configuration, and data
capture from a simulated camera, offering functionalities for capturing raw
image data and encoded images for streaming.
"""

import numpy as np
import torch
import cv2
import base64
import json
import asyncio
import logging
from typing import Optional, Tuple, Dict, Any
import time

# Isaac Sim imports
try:
    from isaacsim.sensors.camera import Camera
    from isaacsim.core.utils.prims import get_prim_at_path
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import UsdGeom
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    try:
        from omni.isaac.sensor import Camera
        from omni.isaac.core.utils.prims import get_prim_at_path
        from omni.isaac.core.utils.stage import get_current_stage
        from pxr import UsdGeom
        ISAAC_SIM_AVAILABLE = True
        logging.info("Using legacy Isaac Sim import paths.")
    except ImportError as e:
        logging.warning(f"Isaac Sim modules not available: {e}. Camera functionality will be disabled.")
        ISAAC_SIM_AVAILABLE = False

# Initialize logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RGBCameraSensor:
    """Manages an RGB camera sensor within the Isaac Sim environment.

    This class encapsulates the logic for creating and controlling a camera,
    capturing RGB images, and providing performance metrics.

    Attributes:
        prim_path (str): The path of the camera prim in the USD stage.
        position (np.ndarray): The 3D position of the camera.
        target (np.ndarray): The 3D position the camera is pointing at.
        resolution (Tuple[int, int]): The (width, height) resolution of the camera.
        frequency (float): The capture frequency in Hz.
        camera (Optional[Camera]): The underlying Isaac Sim Camera object.
        is_initialized (bool): True if the camera has been successfully initialized.
    """
    
    def __init__(self, 
                 prim_path: str = "/World/RGBCamera",
                 position: np.ndarray = np.array([2.0, 2.0, 2.0]),
                 target: np.ndarray = np.array([0.0, 0.0, 0.0]),
                 resolution: Tuple[int, int] = (1280, 720),
                 frequency: float = 30.0):
        """Initializes the RGBCameraSensor.

        Args:
            prim_path (str): The path for the camera prim in the USD stage.
            position (np.ndarray): The 3D world position (x, y, z) of the camera.
            target (np.ndarray): The 3D world position for the camera to look at.
            resolution (Tuple[int, int]): The image resolution (width, height).
            frequency (float): The capture frequency in frames per second.
        """
        self.prim_path = prim_path
        self.position = position
        self.target = target
        self.resolution = resolution
        self.frequency = frequency
        self.camera = None
        self.is_initialized = False
        
        # Performance monitoring
        self.capture_count = 0
        self.last_capture_time = 0
        self.fps_history = []
        
        logger.info(f"🎥 RGBCameraSensor created for prim path: {prim_path}")
        
    def initialize(self) -> bool:
        """Initializes the camera sensor and adds it to the Isaac Sim stage.

        This method creates the underlying Isaac Sim `Camera` object and configures
        its properties.

        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        if not ISAAC_SIM_AVAILABLE:
            logger.error("❌ Cannot initialize camera: Isaac Sim modules are not available.")
            return False
            
        try:
            logger.info("🎬 Initializing RGB camera sensor...")
            self.camera = Camera(
                prim_path=self.prim_path,
                position=self.position,
                look_at=self.target,
                resolution=self.resolution,
                frequency=self.frequency
            )
            self.camera.initialize()
            self._configure_camera_properties()
            self.is_initialized = True
            logger.info("✅ RGB camera sensor initialized successfully.")
            return True
        except Exception as e:
            logger.error(f"❌ Failed to initialize RGB camera: {e}", exc_info=True)
            return False
            
    def _configure_camera_properties(self):
        """Sets detailed properties for the camera prim."""
        if not self.camera:
            return
        try:
            camera_prim = get_current_stage().GetPrimAtPath(self.prim_path)
            if camera_prim:
                camera_api = UsdGeom.Camera(camera_prim)
                camera_api.GetFocalLengthAttr().Set(24.0)
                camera_api.GetFStopAttr().Set(2.8)
                logger.info("🔧 Camera properties configured.")
        except Exception as e:
            logger.warning(f"⚠️ Failed to configure some camera properties: {e}")
            
    def capture_rgb_image(self) -> Optional[np.ndarray]:
        """Captures an RGB image from the sensor.

        Returns:
            Optional[np.ndarray]: The captured RGB image as a NumPy array (H, W, 3),
            or None if the capture fails.
        """
        if not self.is_initialized or not self.camera:
            logger.error("❌ Camera is not initialized.")
            return None
            
        try:
            rgba_data = self.camera.get_rgba()
            if rgba_data is None or not rgba_data.shape[0]:
                logger.warning("⚠️ Could not get image from camera.")
                return None
            
            rgb_data = rgba_data[:, :, :3].copy()
            self._update_performance_metrics()
            logger.debug(f"📸 RGB image captured with shape: {rgb_data.shape}")
            return rgb_data
        except Exception as e:
            logger.error(f"❌ Failed to capture RGB image: {e}", exc_info=True)
            return None
            
    def _update_performance_metrics(self):
        """Updates internal performance metrics like FPS."""
        current_time = time.time()
        if self.last_capture_time > 0:
            fps = 1.0 / (current_time - self.last_capture_time)
            self.fps_history.append(fps)
            if len(self.fps_history) > 30: # Keep last 30 samples
                self.fps_history.pop(0)
        self.last_capture_time = current_time
        self.capture_count += 1
        
    def get_performance_metrics(self) -> Dict[str, Any]:
        """Returns a dictionary of performance metrics.

        Returns:
            Dict[str, Any]: A dictionary containing metrics like average FPS
            and total captures.
        """
        if not self.fps_history:
            return {"fps": 0.0, "capture_count": self.capture_count}
            
        avg_fps = sum(self.fps_history) / len(self.fps_history)
        return {
            "fps": round(avg_fps, 2),
            "capture_count": self.capture_count,
            "target_fps": self.frequency,
            "fps_stability": round(np.std(self.fps_history), 2)
        }
        
    def capture_and_encode_image(self, quality: int = 90, img_format: str = ".jpg") -> Optional[str]:
        """Captures an image and encodes it to a Base64 string.

        Args:
            quality (int): The compression quality for JPEG (1-100).
            img_format (str): The image format to encode to (e.g., ".jpg", ".png").

        Returns:
            Optional[str]: The Base64 encoded image string, or None on failure.
        """
        rgb_image = self.capture_rgb_image()
        if rgb_image is None:
            return None
            
        try:
            bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, quality] if img_format == ".jpg" else []
            _, buffer = cv2.imencode(img_format, bgr_image, encode_param)
            encoded_image = base64.b64encode(buffer).decode('utf-8')
            logger.debug(f"🔐 Image encoded to Base64 (length: {len(encoded_image)}).")
            return encoded_image
        except Exception as e:
            logger.error(f"❌ Failed to encode image: {e}", exc_info=True)
            return None
            
    def get_camera_info(self) -> Dict[str, Any]:
        """Returns a dictionary with the camera's current configuration and status.

        Returns:
            Dict[str, Any]: A dictionary containing camera information.
        """
        return {
            "prim_path": self.prim_path,
            "position": self.position.tolist(),
            "target": self.target.tolist(),
            "resolution": self.resolution,
            "frequency": self.frequency,
            "is_initialized": self.is_initialized,
            "performance": self.get_performance_metrics()
        }
        
    def update_pose(self, position: Optional[np.ndarray] = None, target: Optional[np.ndarray] = None):
        """Updates the camera's position and/or target look_at point.

        Args:
            position (Optional[np.ndarray]): The new camera position.
            target (Optional[np.ndarray]): The new target position to look at.
        """
        if not self.is_initialized or not self.camera:
            logger.warning("⚠️ Camera not initialized, cannot update pose.")
            return
            
        try:
            if position is not None:
                self.position = position
            if target is not None:
                self.target = target

            self.camera.set_world_pose(position=self.position, look_at=self.target)
            logger.info(f"📍 Camera pose updated: pos={self.position}, target={self.target}")
        except Exception as e:
            logger.error(f"❌ Failed to update camera pose: {e}", exc_info=True)
            
    def cleanup(self):
        """Cleans up resources used by the camera."""
        try:
            if self.camera:
                self.camera = None
            self.is_initialized = False
            logger.info("🧹 RGB camera sensor cleaned up.")
        except Exception as e:
            logger.error(f"❌ Failed to clean up camera: {e}", exc_info=True)
            
    def __enter__(self):
        """Enters the context manager, initializing the camera."""
        if self.initialize():
            return self
        raise RuntimeError("Failed to initialize camera.")
            
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Exits the context manager, cleaning up resources."""
        self.cleanup()


def create_rgb_camera(prim_path: str = "/World/RGBCamera",
                     position: Tuple[float, float, float] = (2.0, 2.0, 2.0),
                     target: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                     resolution: Tuple[int, int] = (1280, 720),
                     frequency: float = 30.0) -> RGBCameraSensor:
    """Factory function to create an `RGBCameraSensor`.

    Args:
        prim_path (str): The path for the camera prim in the USD stage.
        position (Tuple[float, float, float]): The 3D world position of the camera.
        target (Tuple[float, float, float]): The 3D world position to look at.
        resolution (Tuple[int, int]): The image resolution (width, height).
        frequency (float): The capture frequency in frames per second.

    Returns:
        RGBCameraSensor: An uninitialized `RGBCameraSensor` instance.
    """

    return RGBCameraSensor(
        prim_path=prim_path,
        position=np.array(position),
        target=np.array(target),
        resolution=resolution,
        frequency=frequency
    )


if __name__ == "__main__":
    logger.info("🧪 Testing RGB Camera Sensor module...")
    
    if not ISAAC_SIM_AVAILABLE:
        logger.warning("⚠️ Isaac Sim not available. Running a limited test.")

    # Test camera sensor creation
    camera_sensor = create_rgb_camera()
    camera_info = camera_sensor.get_camera_info()
    
    logger.info(f"📋 Camera Info: {json.dumps(camera_info, indent=2)}")
    logger.info("🎉 RGB Camera Sensor module test finished.")