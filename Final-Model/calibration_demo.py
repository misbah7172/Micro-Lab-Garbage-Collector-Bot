#!/usr/bin/env python3
"""
Camera Distance Calibration Tool
=================================
Dedicated tool for calibrating camera distance measurement.
Creates calibration data that will be used by the main robot system.

Usage:
1. Run this script: python calibration_demo.py
2. Press 'c' to enter calibration mode
3. Place a 10x10 cm object in camera view
4. Press 's' to save calibration sample
5. Enter actual distance in terminal
6. Repeat steps 3-5 for different distances (recommended: 3-5 samples)
7. Press 'q' to quit
8. Run enhanced_autonomous_robot.py to use the calibration

Saves calibration data to: camera_calibration.json

Author: Micro Lab Garbage Collector Team
Date: September 2025
"""

import sys
import os
import time

# Add the path to import the robot module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from enhanced_autonomous_robot import EnhancedAutonomousRobot

def main():
    print("🔧 Camera Distance Calibration Tool")
    print("=" * 50)
    print()
    print("📋 This tool creates calibration data for distance measurement")
    print("📁 Calibration will be saved to: camera_calibration.json")
    print()
    print("📋 Instructions:")
    print("1. Camera window will open")
    print("2. Press 'c' to enter calibration mode")
    print("3. Place a 10x10 cm object in camera view")
    print("4. Press 's' to save calibration sample")
    print("5. Enter actual distance in this terminal")
    print("6. Repeat for different distances (3-5 samples recommended)")
    print("7. Press 'q' to quit and save calibration")
    print()
    print("💡 After calibration, run enhanced_autonomous_robot.py")
    print("   The main robot will automatically load this calibration!")
    print()
    print("Starting calibration system...")
    
    # Create robot instance (camera and AI only)
    robot = EnhancedAutonomousRobot()
    calibration_count = 0
    
    try:
        # Initialize camera and AI model only
        if not robot.setup_camera():
            print("❌ Camera setup failed")
            return 1
            
        if not robot.load_ai_model():
            print("❌ AI model loading failed")
            return 1
            
        print("✅ Camera and AI model ready")
        print("📺 Camera window should now be open")
        print()
        
        # Run vision processing loop
        robot.running = True
        
        while robot.running:
            try:
                if robot.camera:
                    ret, frame = robot.camera.read()
                    if ret:
                        has_objects, annotated_frame = robot.detect_objects_enhanced(frame)
                        
                        # Display frame
                        if robot.config.SHOW_CAMERA_FEED:
                            import cv2
                            cv2.imshow('Camera Distance Calibration Demo', annotated_frame)
                            
                            # Handle keyboard input
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                break
                            elif key == ord('c') and robot.config.ENABLE_CAMERA_DISTANCE:
                                # Toggle calibration mode
                                robot.calibration_mode = not robot.calibration_mode
                                if robot.calibration_mode:
                                    robot.calibration_samples = []
                                    print(f"\n📏 Calibration mode ENABLED")
                                    print(f"   - Place {robot.config.CALIBRATION_OBJECT_WIDTH}x{robot.config.CALIBRATION_OBJECT_HEIGHT}cm object at known distance")
                                    print(f"   - Press 's' to save calibration sample")
                                else:
                                    print("\n📏 Calibration mode DISABLED")
                            elif key == ord('s') and robot.calibration_mode:
                                # Save calibration sample
                                if robot.detection_data.object_count > 0:
                                    bbox_width = robot.detection_data.bbox_width
                                    bbox_height = robot.detection_data.bbox_height
                                    
                                    print(f"\n📏 Calibration Sample #{calibration_count + 1}")
                                    print(f"   Bounding box: {bbox_width:.1f} x {bbox_height:.1f} pixels")
                                    
                                    try:
                                        distance_input = input("   Enter actual distance (cm): ")
                                        distance = float(distance_input)
                                        
                                        # Perform calibration
                                        robot.calibrate_camera_distance(bbox_width, bbox_height, distance)
                                        calibration_count += 1
                                        
                                        print(f"   ✅ Sample #{calibration_count} saved at {distance:.1f}cm")
                                        
                                        # Test distance measurement
                                        estimated = robot.calculate_distance_from_bbox(bbox_width, bbox_height)
                                        error = abs(estimated - distance)
                                        error_percent = (error / distance) * 100
                                        print(f"   🎯 Distance estimation: {estimated:.1f}cm (error: {error:.1f}cm, {error_percent:.1f}%)")
                                        
                                        if calibration_count >= 3:
                                            print(f"\n✅ Good! You have {calibration_count} calibration samples.")
                                            print("   💡 You can add more samples or press 'q' to finish.")
                                        
                                    except ValueError:
                                        print("   ❌ Invalid distance. Please enter a number.")
                                    except Exception as e:
                                        print(f"   ❌ Calibration error: {e}")
                                else:
                                    print("\n⚠️ No object detected for calibration")
                        
                time.sleep(1.0 / 30)  # 30 FPS
                
            except KeyboardInterrupt:
                print("\n🛑 Stopping calibration demo...")
                break
                
    except Exception as e:
        print(f"❌ Demo error: {e}")
        return 1
        
    finally:
        # Cleanup
        robot.running = False
        if robot.camera:
            robot.camera.release()
        import cv2
        cv2.destroyAllWindows()
        
        # Show final calibration status
        print(f"\n📊 Calibration Summary:")
        print(f"   Samples collected: {calibration_count}")
        
        if robot.calibration_data.is_calibrated and calibration_count > 0:
            print("✅ Camera is calibrated for distance measurement!")
            print(f"   📁 Saved to: camera_calibration.json")
            print(f"   🔧 Focal length X: {robot.calibration_data.focal_length_x:.2f}")
            print(f"   🔧 Focal length Y: {robot.calibration_data.focal_length_y:.2f}")
            print(f"   📐 Object size: {robot.calibration_data.known_object_width}x{robot.calibration_data.known_object_height}cm")
            print()
            print("🚀 Next step: Run enhanced_autonomous_robot.py")
            print("   The main robot will automatically load this calibration!")
        else:
            print("⚠️ Camera not calibrated")
            print("   No calibration samples were saved.")
            print("   Try running the calibration again.")
            
        print("\n🏁 Calibration tool complete!")
        
    return 0

if __name__ == "__main__":
    exit(main())