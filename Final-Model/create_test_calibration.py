#!/usr/bin/env python3
"""
Manual Calibration Test
======================
Creates a test calibration file manually to verify the system works.
"""

import json

def create_test_calibration():
    """Create a test calibration file"""
    print("🔧 Creating test calibration file...")
    
    # Test calibration data (example values)
    calibration_data = {
        "known_object_width": 10.0,
        "known_object_height": 10.0,
        "known_distance": 50.0,
        "pixel_width": 200.0,
        "pixel_height": 150.0,
        "focal_length_x": 100.0,  # (pixel_width * known_distance) / known_object_width
        "focal_length_y": 75.0,   # (pixel_height * known_distance) / known_object_height
        "is_calibrated": True,
        "calibration_timestamp": 1695481200.0
    }
    
    try:
        with open('camera_calibration.json', 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print("✅ Test calibration file created!")
        print(f"   📁 File: camera_calibration.json")
        print(f"   📏 Object: {calibration_data['known_object_width']}x{calibration_data['known_object_height']}cm")
        print(f"   📐 Distance: {calibration_data['known_distance']}cm")
        print(f"   🔧 Focal Length: X={calibration_data['focal_length_x']}, Y={calibration_data['focal_length_y']}")
        print(f"   ✅ Calibrated: {calibration_data['is_calibrated']}")
        print()
        print("🚀 Now run: python enhanced_autonomous_robot.py")
        print("   Distance measurements should work!")
        
    except Exception as e:
        print(f"❌ Error creating calibration file: {e}")

if __name__ == "__main__":
    create_test_calibration()