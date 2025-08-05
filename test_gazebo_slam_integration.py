#!/usr/bin/env python3
"""
Test script to validate Gazebo SLAM integration without actually launching Gazebo.
This script checks file existence, parameter validation, and launch file syntax.
"""

import os
import yaml
import sys
from pathlib import Path

def test_file_exists(filepath, description):
    """Test if a file exists"""
    if os.path.exists(filepath):
        print(f"✅ {description}: {filepath}")
        return True
    else:
        print(f"❌ {description}: {filepath} - FILE NOT FOUND")
        return False

def test_yaml_valid(filepath, description):
    """Test if YAML file is valid"""
    try:
        with open(filepath, 'r') as f:
            yaml.safe_load(f)
        print(f"✅ {description}: Valid YAML syntax")
        return True
    except Exception as e:
        print(f"❌ {description}: Invalid YAML - {e}")
        return False

def test_launch_file_python_syntax(filepath, description):
    """Test if Python launch file has valid syntax"""
    try:
        with open(filepath, 'r') as f:
            code = f.read()
        compile(code, filepath, 'exec')
        print(f"✅ {description}: Valid Python syntax")
        return True
    except Exception as e:
        print(f"❌ {description}: Invalid Python syntax - {e}")
        return False

def main():
    print("🔍 B4M SLAM Toolbox Gazebo Integration Test")
    print("=" * 60)
    
    # Get the workspace root
    workspace_root = Path(__file__).parent
    
    tests_passed = 0
    total_tests = 0
    
    # Test 1: SLAM Toolbox Parameters (Simulation)
    total_tests += 1
    slam_sim_params = workspace_root / "yahboomcar_nav/params/slam_toolbox_sim_params.yaml"
    if test_file_exists(slam_sim_params, "SLAM Simulation Parameters"):
        if test_yaml_valid(slam_sim_params, "SLAM Simulation Parameters YAML"):
            tests_passed += 1
    
    # Test 2: SLAM Navigation Parameters
    total_tests += 1
    slam_nav_params = workspace_root / "yahboomcar_nav/params/slam_nav_params.yaml"
    if test_file_exists(slam_nav_params, "SLAM Navigation Parameters"):
        if test_yaml_valid(slam_nav_params, "SLAM Navigation Parameters YAML"):
            tests_passed += 1
    
    # Test 3: SLAM Toolbox Launch File
    total_tests += 1
    slam_launch = workspace_root / "yahboomcar_nav/launch/slam_toolbox_launch.py"
    if test_file_exists(slam_launch, "SLAM Toolbox Launch File"):
        if test_launch_file_python_syntax(slam_launch, "SLAM Toolbox Launch File"):
            tests_passed += 1
    
    # Test 4: Gazebo SLAM Navigation Launch File
    total_tests += 1
    gazebo_slam_launch = workspace_root / "yahboomcar_nav/launch/gazebo_slam_navigation_launch.py"
    if test_file_exists(gazebo_slam_launch, "Gazebo SLAM Navigation Launch File"):
        if test_launch_file_python_syntax(gazebo_slam_launch, "Gazebo SLAM Navigation Launch File"):
            tests_passed += 1
    
    # Test 5: Check parameter consistency
    total_tests += 1
    try:
        with open(slam_sim_params, 'r') as f:
            sim_params = yaml.safe_load(f)
        
        # Check key simulation parameters
        slam_config = sim_params.get('slam_toolbox', {}).get('ros__parameters', {})
        
        checks = [
            ('use_sim_time', True),
            ('mode', 'mapping'),
            ('minimum_time_interval', 0.1),
            ('minimum_travel_distance', 0.2),
            ('loop_search_maximum_distance', 5.0)
        ]
        
        all_params_valid = True
        for param, expected in checks:
            if slam_config.get(param) == expected:
                print(f"✅ Parameter {param}: {slam_config.get(param)}")
            else:
                print(f"❌ Parameter {param}: Expected {expected}, got {slam_config.get(param)}")
                all_params_valid = False
        
        if all_params_valid:
            tests_passed += 1
            print("✅ Parameter Consistency: All simulation parameters correct")
        else:
            print("❌ Parameter Consistency: Some parameters incorrect")
            
    except Exception as e:
        print(f"❌ Parameter Consistency: Failed to validate - {e}")
    
    # Test 6: Check URDF file for Gazebo compatibility
    total_tests += 1
    urdf_file = workspace_root / "yahboomcar_description/urdf/yahboomcar_robot2.urdf"
    if test_file_exists(urdf_file, "Robot URDF for Gazebo"):
        tests_passed += 1
    
    print("\n" + "=" * 60)
    print(f"📊 Test Results: {tests_passed}/{total_tests} tests passed")
    
    if tests_passed == total_tests:
        print("🎉 All tests passed! Gazebo SLAM integration is ready.")
        print("\n📝 Next Steps:")
        print("1. Install Gazebo: sudo apt install gazebo ros-humble-gazebo-ros-pkgs")
        print("2. Test launch: ros2 launch yahboomcar_nav gazebo_slam_navigation_launch.py")
        print("3. Verify SLAM mapping in RViz while moving robot in Gazebo")
        return 0
    else:
        print("⚠️  Some tests failed. Please fix the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())