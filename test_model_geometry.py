#!/usr/bin/env python3
# test_model_geometry.py

import xml.etree.ElementTree as ET
import math
import sys

def test_urdf_geometry():
    """Test 1: Validate URDF geometry and calculate ground contact"""
    urdf_path = "/home/mike/projects/b4m_yahboom/yahboomcar_description/urdf/yahboomcar_robot2_gazebo.urdf"
    
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        # Extract key measurements
        base_joint = root.find(".//joint[@name='base_joint']/origin")
        base_z = float(base_joint.get('xyz').split()[2])
        print(f"Base joint Z offset: {base_z}m")
        
        wheel_joints = {}
        for wheel in ['left_front', 'right_front', 'left_back', 'right_back']:
            joint = root.find(f".//joint[@name='{wheel}_joint']/origin")
            if joint is not None:
                xyz = joint.get('xyz').split()
                wheel_joints[wheel] = {
                    'x': float(xyz[0]),
                    'y': float(xyz[1]),
                    'z': float(xyz[2])
                }
                print(f"{wheel}_joint position: x={wheel_joints[wheel]['x']}, y={wheel_joints[wheel]['y']}, z={wheel_joints[wheel]['z']}")
        
        # Get wheel radius from DiffDrive plugin
        diff_drive = root.find(".//plugin[@name='ignition::gazebo::systems::DiffDrive']")
        wheel_radius_elem = diff_drive.find('wheel_radius')
        wheel_radius = float(wheel_radius_elem.text)
        print(f"Wheel radius from DiffDrive: {wheel_radius}m")
        
        # Check collision geometry
        cylinders = root.findall(".//collision/geometry/cylinder")
        print(f"\nFound {len(cylinders)} cylinder collision geometries")
        for i, cyl in enumerate(cylinders):
            radius = float(cyl.get('radius'))
            length = float(cyl.get('length'))
            print(f"  Cylinder {i+1}: radius={radius}m, length={length}m")
        
        # Calculate expected ground contact
        print("\nGround contact analysis:")
        all_good = True
        for wheel, pos in wheel_joints.items():
            ground_clearance = base_z + pos['z'] - wheel_radius
            print(f"{wheel}: Expected ground clearance = {ground_clearance*1000:.1f}mm")
            if ground_clearance > 0.001:
                print(f"  WARNING: {wheel} not touching ground! Clearance: {ground_clearance}m")
                all_good = False
            else:
                print(f"  ✓ {wheel} should be in contact with ground")
        
        print(f"\nTest result: {'PASSED' if all_good else 'FAILED'}")
        return 0 if all_good else 1
        
    except Exception as e:
        print(f"Error during test: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(test_urdf_geometry())