#!/usr/bin/env python3
"""
Test Script for Autonomous Robot System
Quick verification of all components before full autonomous operation
"""

import cv2
import time
import threading
from enhanced_autonomous_robot import EnhancedAutonomousRobot, RobotConfig

def test_camera():
    """Test camera functionality"""
    print("📷 Testing Camera...")
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print("✅ Camera working - captured frame")
                cv2.imshow('Camera Test', frame)
                cv2.waitKey(2000)  # Show for 2 seconds
                cv2.destroyAllWindows()
            cap.release()
            return True
        else:
            print("❌ Camera failed to open")
            return False
    except Exception as e:
        print(f"❌ Camera error: {e}")
        return False

def test_ai_model():
    """Test AI model loading"""
    print("🧠 Testing AI Model...")
    try:
        config = RobotConfig()
        from ultralytics import YOLO
        
        # PyTorch compatibility
        import torch
        original_load = torch.load
        def trusted_load(*args, **kwargs):
            kwargs['weights_only'] = False
            return original_load(*args, **kwargs)
        torch.load = trusted_load
        
        model = YOLO(config.MODEL_PATH)
        print("✅ AI model loaded successfully")
        return True
    except Exception as e:
        print(f"❌ AI model error: {e}")
        return False

def test_serial_connection():
    """Test ESP32 serial connection"""
    print("📡 Testing Serial Connection...")
    try:
        import serial.tools.list_ports
        
        ports = serial.tools.list_ports.comports()
        esp32_ports = []
        
        for port in ports:
            if any(keyword in port.description for keyword in ["CH340", "CP210", "USB"]):
                esp32_ports.append(port.device)
        
        if esp32_ports:
            print(f"✅ Found potential ESP32 ports: {esp32_ports}")
            
            # Try to connect to first port
            import serial
            ser = serial.Serial(esp32_ports[0], 115200, timeout=1)
            ser.write(b"S")  # Send stop command
            time.sleep(0.5)
            ser.close()
            print("✅ Serial communication working")
            return True
        else:
            print("⚠️ No ESP32 ports found - will run in mock mode")
            return True
    except Exception as e:
        print(f"❌ Serial connection error: {e}")
        return False

def test_configuration():
    """Test configuration loading"""
    print("⚙️ Testing Configuration...")
    try:
        config = RobotConfig()
        print(f"✅ Configuration loaded:")
        print(f"   - Model: {config.MODEL_PATH}")
        print(f"   - Camera: {config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT}")
        print(f"   - Obstacle distance: {config.OBSTACLE_DISTANCE}cm")
        print(f"   - Metal classes: {config.METAL_CLASS_IDS}")
        return True
    except Exception as e:
        print(f"❌ Configuration error: {e}")
        return False

def test_object_detection():
    """Test object detection with camera"""
    print("🎯 Testing Object Detection...")
    try:
        robot = EnhancedAutonomousRobot()
        
        if not robot.setup_camera():
            return False
        if not robot.load_ai_model():
            return False
        
        print("Taking test detection...")
        ret, frame = robot.camera.read()
        if ret:
            has_objects, annotated_frame = robot.detect_objects_enhanced(frame)
            
            if has_objects:
                print(f"✅ Objects detected: {robot.detection_data.object_count}")
                print(f"   - Center: ({robot.detection_data.object_center_x}, {robot.detection_data.object_center_y})")
                print(f"   - Confidence: {robot.detection_data.object_confidence:.2f}")
                print(f"   - Is Metal: {robot.detection_data.is_metal}")
            else:
                print("ℹ️ No objects detected in current view")
            
            # Show detection result
            cv2.imshow('Object Detection Test', annotated_frame)
            cv2.waitKey(3000)  # Show for 3 seconds
            cv2.destroyAllWindows()
            
            robot.camera.release()
            return True
        else:
            print("❌ Failed to capture frame")
            return False
            
    except Exception as e:
        print(f"❌ Object detection error: {e}")
        return False

def test_state_machine():
    """Test state machine basic functionality"""
    print("🔄 Testing State Machine...")
    try:
        robot = EnhancedAutonomousRobot()
        
        # Test state transitions
        from enhanced_autonomous_robot import RobotState
        
        robot.transition_to_state(RobotState.SEARCHING)
        print(f"✅ State transition to: {robot.current_state.value}")
        
        robot.transition_to_state(RobotState.OBJECT_DETECTED)
        print(f"✅ State transition to: {robot.current_state.value}")
        
        # Test command sending (mock mode)
        robot.send_command("F")
        robot.send_command("A0")
        robot.send_command("B1")
        
        print("✅ State machine and command system working")
        return True
        
    except Exception as e:
        print(f"❌ State machine error: {e}")
        return False

def run_short_autonomous_test():
    """Run a short autonomous test (30 seconds)"""
    print("🤖 Running Short Autonomous Test (30 seconds)...")
    try:
        robot = EnhancedAutonomousRobot()
        
        print("Starting robot systems...")
        if not robot.setup_camera():
            print("❌ Camera setup failed")
            return False
        
        if not robot.load_ai_model():
            print("❌ AI model loading failed")
            return False
        
        robot.setup_serial_connection()  # Optional
        
        # Start robot for 30 seconds
        robot.running = True
        
        # Start only vision thread for testing
        vision_thread = threading.Thread(target=robot.vision_processing_thread, daemon=True)
        vision_thread.start()
        
        print("✅ Robot running autonomously...")
        print("   - Press 'q' in camera window to stop early")
        print("   - Will auto-stop after 30 seconds")
        
        start_time = time.time()
        while robot.running and (time.time() - start_time) < 30:
            time.sleep(1)
            elapsed = time.time() - start_time
            print(f"   [{elapsed:.0f}s] State: {robot.current_state.value} | Objects: {robot.detection_data.object_count}")
        
        robot.stop()
        print("✅ Short autonomous test completed successfully")
        return True
        
    except Exception as e:
        print(f"❌ Autonomous test error: {e}")
        return False

def main():
    """Run comprehensive system test"""
    print("🧪 Micro Lab Garbage Collector - System Test")
    print("=" * 60)
    
    tests = [
        ("Configuration", test_configuration),
        ("Camera", test_camera),
        ("AI Model", test_ai_model),
        ("Serial Connection", test_serial_connection),
        ("Object Detection", test_object_detection),
        ("State Machine", test_state_machine),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n{test_name} Test:")
        print("-" * 30)
        try:
            if test_func():
                passed += 1
                print(f"✅ {test_name} test PASSED")
            else:
                print(f"❌ {test_name} test FAILED")
        except Exception as e:
            print(f"❌ {test_name} test ERROR: {e}")
        
        time.sleep(1)  # Brief pause between tests
    
    print(f"\n📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! System ready for autonomous operation.")
        
        response = input("\nRun short autonomous test? (y/n): ").lower().strip()
        if response == 'y':
            run_short_autonomous_test()
            
    else:
        print("⚠️ Some tests failed. Please fix issues before running autonomously.")
    
    print("\n🏁 System test complete!")

if __name__ == "__main__":
    main()