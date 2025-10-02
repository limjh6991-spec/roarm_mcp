#!/usr/bin/env python3
"""
====================================================================
=== 기본 Isaac Sim 로봇 USD 구조 분석 스크립트 ===
====================================================================

Isaac Sim 표준 import 경로를 사용하여 로봇 USD 구조를 분석하고
SingleArticulation 경로를 찾는 간소화된 접근
"""

import sys
import os

# Isaac Sim 환경 설정 확인
print("🔍 Isaac Sim 환경 확인...")
print(f"Python 경로: {sys.executable}")
print(f"작업 디렉토리: {os.getcwd()}")

# 기본 Isaac Sim 경로 체크
isaac_sim_paths = [
    "/opt/nvidia/omniverse/isaac-sim",
    "/isaac-sim", 
    "~/.local/share/ov/pkg/isaac_sim-*",
]

for path in isaac_sim_paths:
    expanded_path = os.path.expanduser(path)
    print(f"경로 확인: {expanded_path}")
    if os.path.exists(expanded_path):
        print(f"  ✅ 발견")
    else:
        print(f"  ❌ 없음")

# USD 파일 직접 분석 시도
print("\n🧩 Isaac Sim USD 로봇 파일 직접 분석...")

# 가능한 로봇 USD 파일 경로들
robot_usd_paths = [
    "/Isaac/Robots/UniversalRobots/ur10/ur10.usd",
    "/Isaac/Robots/Franka/franka.usd", 
    "/Isaac/Robots/Franka/franka_alt_fingers.usd",
]

print(f"분석할 로봇 USD 파일들: {robot_usd_paths}")

try:
    # pxr USD 라이브러리로 파일 구조 분석
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf
    
    print("✅ USD 라이브러리 import 성공")
    
    for usd_path in robot_usd_paths:
        print(f"\n📄 {usd_path} 분석 중...")
        
        try:
            # USD 스테이지 열기
            stage = Usd.Stage.Open(usd_path)
            if not stage:
                print(f"  ❌ USD 파일을 열 수 없습니다: {usd_path}")
                continue
                
            print(f"  ✅ USD 파일 열기 성공")
            
            # 모든 프림 순회
            print(f"  📊 프림 구조 분석:")
            
            articulation_roots = []
            joints = []
            links = []
            
            for prim in stage.Traverse():
                prim_path = prim.GetPath().pathString
                prim_type = prim.GetTypeName()
                
                # 깊이 제한 (너무 깊지 않은 주요 구조만)
                depth = len(prim_path.split('/')) - 1
                if depth <= 4:
                    indent = "    " * depth
                    print(f"{indent}{prim_path} ({prim_type})")
                
                # ArticulationRoot API 확인
                try:
                    articulation_root_api = UsdPhysics.ArticulationRootAPI(prim)
                    if articulation_root_api:
                        articulation_roots.append(prim_path)
                        print(f"{indent}  🎯 ArticulationRoot API 발견!")
                except:
                    pass
                
                # 관절 타입 확인
                if prim_type in ["Joint", "RevoluteJoint", "PrismaticJoint"]:
                    joints.append(prim_path)
                    print(f"{indent}  🔗 관절 발견: {prim_type}")
                
                # 링크 패턴 확인
                if "link" in prim_path.lower() or "base" in prim_path.lower():
                    links.append(prim_path)
                    print(f"{indent}  🔧 링크 후보: {prim_path}")
            
            # 분석 결과 요약
            print(f"\n  📈 분석 결과 요약:")
            print(f"    - ArticulationRoot: {len(articulation_roots)}개")
            print(f"    - 관절: {len(joints)}개")  
            print(f"    - 링크 후보: {len(links)}개")
            
            if articulation_roots:
                print(f"    🎯 ArticulationRoot 경로들:")
                for root in articulation_roots:
                    print(f"      - {root}")
            else:
                print(f"    ⚠️ ArticulationRoot를 찾지 못했습니다")
                
            if joints:
                print(f"    🔗 관절 경로들 (처음 5개):")
                for joint in joints[:5]:
                    print(f"      - {joint}")
                if len(joints) > 5:
                    print(f"      ... 및 {len(joints)-5}개 더")
                    
        except Exception as e:
            print(f"  ❌ USD 파일 분석 실패: {e}")
            import traceback
            traceback.print_exc()
            
except ImportError as e:
    print(f"❌ USD 라이브러리 import 실패: {e}")
    print("\nIsaac Sim 환경이 제대로 설정되지 않았을 수 있습니다.")
    print("다음 명령어들을 시도해보세요:")
    print("1. source ~/.bashrc")
    print("2. Isaac Sim을 직접 실행한 후 스크립트 모드로 전환")

except Exception as e:
    print(f"❌ 일반적인 오류 발생: {e}")
    import traceback
    traceback.print_exc()

print("\n🏁 USD 구조 분석 완료")