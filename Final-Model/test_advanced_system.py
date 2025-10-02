#!/usr/bin/env python3

import os
import sys
import time
import serial
import serial.tools.list_ports
import cv2

def test_camera():
    """Test camera connectivity"""
    print("🔍 Testing camera...")
    try:
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            print("❌ Camera not found")
            return False
        
        ret, frame = camera.read()
        if ret:
            print(f"✅ Camera working - Frame size: {frame.shape}")
            camera.release()
            return True
        else:
            print("❌ Camera not capturing frames")
            camera.release()
            return False
    except Exception as e:
        print(f"❌ Camera error: {e}")
        return False

def test_serial():
    """Test ESP32 serial connection"""
    print("🔌 Testing ESP32 connection...")
    try:
        # Find Arduino/ESP32 port
        arduino_keywords = ['Arduino', 'ESP32', 'CH340', 'CP210', 'USB-SERIAL']
        arduino_port = None
        
        for port in serial.tools.list_ports.comports():
            port_description = f"{port.description} {port.manufacturer or ''}"
            if any(keyword.lower() in port_description.lower() for keyword in arduino_keywords):
                arduino_port = port.device
                break
        
        if not arduino_port:
            print("❌ ESP32 not found")
            print("Available ports:")
            for port in serial.tools.list_ports.comports():
                print(f"  - {port.device}: {port.description}")
            return False
        
        # Test connection
        ser = serial.Serial(arduino_port, 115200, timeout=2)
        time.sleep(2)  # Wait for ESP32 to initialize
        
        ser.write(b"HEARTBEAT\n")
        time.sleep(0.5)
        
        response = ""
        while ser.in_waiting:
            response += ser.read(ser.in_waiting).decode()
        
        ser.close()
        
        print(f"✅ ESP32 connected on {arduino_port}")
        if response:
            print(f"📡 Response: {response.strip()}")
        return True
        
    except Exception as e:
        print(f"❌ ESP32 connection error: {e}")
        return False

def test_yolo_model():
    """Test YOLO model loading"""
    print("🧠 Testing AI model...")
    try:
        from ultralytics import YOLO
        
        model_files = ['best.pt', 'yolov8n.pt', 'yolov8s.pt']
        model_loaded = False
        
        for model_file in model_files:
            if os.path.exists(model_file):
                try:
                    model = YOLO(model_file)
                    print(f"✅ Model loaded: {model_file}")
                    model_loaded = True
                    break
                except Exception as e:
                    print(f"⚠️  Failed to load {model_file}: {e}")
        
        if not model_loaded:
            print("❌ No working YOLO model found")
            print("Available model files:")
            for file in os.listdir('.'):
                if file.endswith('.pt'):
                    print(f"  - {file}")
            return False
        
        return True
        
    except ImportError:
        print("❌ ultralytics not installed - run: pip install ultralytics")
        return False
    except Exception as e:
        print(f"❌ Model loading error: {e}")
        return False

def test_calibration():
    """Test camera calibration"""
    print("📏 Testing camera calibration...")
    try:
        import json
        
        if os.path.exists('camera_calibration.json'):
            with open('camera_calibration.json', 'r') as f:
                cal_data = json.load(f)
            
            if cal_data.get('is_calibrated', False):
                print("✅ Camera calibration found and valid")
                print(f"   Focal length: {cal_data.get('focal_length_x', 0):.1f}")
                return True
            else:
                print("⚠️  Calibration file found but not validated")
                return False
        else:
            print("❌ No calibration file found")
            print("   Run: python calibration_demo.py")
            return False
            
    except Exception as e:
        print(f"❌ Calibration error: {e}")
        return False

def test_dependencies():
    """Test Python dependencies"""
    print("📦 Testing Python dependencies...")
    
    required_packages = [
        ('cv2', 'opencv-python'),
        ('serial', 'pyserial'),
        ('numpy', 'numpy'),
        ('ultralytics', 'ultralytics'),
        ('supervision', 'supervision'),
    ]
    
    missing = []
    
    for package, pip_name in required_packages:
        try:
            __import__(package)
            print(f"✅ {package}")
        except ImportError:
            print(f"❌ {package} - install with: pip install {pip_name}")
            missing.append(pip_name)
    
    if missing:
        print(f"\n💡 Install missing packages: pip install {' '.join(missing)}")
        return False
    
    return True

def main():
    """Run complete system test"""
    print("🤖 Advanced Autonomous Robot - System Test")
    print("="*60)
    
    tests = [
        ("Dependencies", test_dependencies),
        ("Camera", test_camera),
        ("ESP32 Serial", test_serial),
        ("AI Model", test_yolo_model),
        ("Calibration", test_calibration),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n📋 Testing {test_name}...")
        if test_func():
            passed += 1
        else:
            print(f"💥 {test_name} test failed")
    
    print("\n" + "="*60)
    print(f"📊 Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 All tests passed! System ready for operation.")
        print("\n🚀 To start the robot:")
        print("   python advanced_autonomous_robot.py")
    else:
        print("⚠️  Some tests failed. Please fix issues before running.")
        print("\n🔧 Common fixes:")
        print("   - Install missing dependencies")
        print("   - Connect ESP32 via USB")
        print("   - Connect camera")
        print("   - Run calibration: python calibration_demo.py")
    
    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)