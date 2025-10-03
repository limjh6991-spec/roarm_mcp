"""
Sensors Package for RoArm MCP
로봇 센서 시스템 통합 패키지

이 패키지는 Isaac Sim 5.0과 연동하여 다양한 센서 데이터를
수집하고 MCP 프로토콜로 전송하는 기능을 제공합니다.

Modules:
- camera_sensor: RGB/Depth 카메라 센서
- lidar_sensor: LiDAR 센서 (미래 구현)
- imu_sensor: IMU 센서 (미래 구현)
"""

from .camera_sensor import RGBCameraSensor, create_rgb_camera

__all__ = [
    'RGBCameraSensor',
    'create_rgb_camera'
]

__version__ = "0.1.0"