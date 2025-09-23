#!/usr/bin/env python3
"""
Distance Measurement Troubleshooting Guide
==========================================
This script helps diagnose issues with camera distance measurement.
"""

import json
import os

def check_calibration_status():
    """Check the current calibration status and provide diagnostics"""
    print("🔍 Distance Measurement Troubleshooting")
    print("=" * 50)
    print()
    
    calibration_file = "camera_calibration.json"
    
    # Check if calibration file exists
    if not os.path.exists(calibration_file):
        print("❌ ISSUE: No calibration file found")
        print(f"   📁 Looking for: {calibration_file}")
        print("   💡 Solution: Run 'python calibration_demo.py' first")
        print()
        return False
    
    print(f"✅ Calibration file found: {calibration_file}")
    
    # Try to load and validate calibration data
    try:
        with open(calibration_file, 'r') as f:
            data = json.load(f)
        
        print("✅ Calibration file is valid JSON")
        print()
        print("📊 Calibration Data:")
        
        required_fields = [
            'known_object_width', 'known_object_height', 'known_distance',
            'pixel_width', 'pixel_height', 'focal_length_x', 'focal_length_y',
            'is_calibrated'
        ]
        
        missing_fields = []
        for field in required_fields:
            if field in data:
                value = data[field]
                print(f"   ✅ {field}: {value}")
                
                # Check for reasonable values
                if field in ['focal_length_x', 'focal_length_y'] and value <= 0:
                    print(f"      ⚠️ Warning: {field} should be > 0")
                elif field == 'is_calibrated' and not value:
                    print(f"      ⚠️ Warning: is_calibrated is False")
            else:
                missing_fields.append(field)
                print(f"   ❌ {field}: MISSING")
        
        print()
        
        if missing_fields:
            print("❌ ISSUE: Missing required calibration fields")
            print(f"   Missing: {missing_fields}")
            print("   💡 Solution: Delete calibration file and recalibrate")
            return False
        
        if not data.get('is_calibrated', False):
            print("❌ ISSUE: is_calibrated is False")
            print("   💡 Solution: Complete calibration process")
            return False
        
        focal_x = data.get('focal_length_x', 0)
        focal_y = data.get('focal_length_y', 0)
        if focal_x <= 0 or focal_y <= 0:
            print("❌ ISSUE: Invalid focal length values")
            print("   💡 Solution: Recalibrate with proper measurements")
            return False
        
        print("✅ Calibration data appears valid!")
        print()
        print("🎯 Expected distance calculation:")
        print(f"   For 200x150 pixel object: {(focal_x * 10.0) / 200.0:.1f}cm (width) | {(focal_y * 10.0) / 150.0:.1f}cm (height)")
        print()
        print("🚀 If robot still shows 999.0 cm:")
        print("   1. Check ENABLE_CAMERA_DISTANCE=True in robot_config.env")
        print("   2. Ensure object is detected (green bounding box)")
        print("   3. Verify 'Distance: CALIBRATED' appears in robot window")
        
        return True
        
    except json.JSONDecodeError as e:
        print("❌ ISSUE: Calibration file is corrupted")
        print(f"   Error: {e}")
        print("   💡 Solution: Delete file and recalibrate")
        print(f"   Command: del {calibration_file}")
        return False
        
    except Exception as e:
        print(f"❌ ISSUE: Unexpected error reading calibration")
        print(f"   Error: {e}")
        return False

def main():
    success = check_calibration_status()
    
    print()
    if success:
        print("🎉 Calibration system appears to be working correctly!")
        print("   Run: python enhanced_autonomous_robot.py")
    else:
        print("🔧 Follow the solutions above to fix calibration issues")
        print("   Then run: python calibration_demo.py")

if __name__ == "__main__":
    main()