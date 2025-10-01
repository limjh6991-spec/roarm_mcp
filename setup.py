"""
설치 스크립트: Robot Arm Model Context Protocol
"""

from setuptools import setup, find_packages

setup(
    name="roarm_mcp",
    version="0.1.0",
    description="NVIDIA Isaac Sim을 사용한 로봇 팔 강화 학습을 위한 Model Context Protocol 환경",
    author="RoArm Team",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy>=1.20.0",
        "websockets>=10.4",
        "pydantic>=1.10.0",
        "gymnasium>=0.28.0",
        "stable-baselines3>=2.0.0", 
        "torch>=1.13.0",
        # Isaac Sim 패키지는 별도로 설치해야 함
        # "omni-isaac-sim",  # NVIDIA Isaac Sim에서 제공
    ],
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
)