# 🤖 Micro Lab Garbage Collector - Usage Guide

## 📁 **Which File to Run?**

### 🎯 **Main Project Files:**

| File | Purpose | When to Use |
|------|---------|-------------|
| **`enhanced_autonomous_robot.py`** | **🚀 MAIN ROBOT SYSTEM** | **Run this for full autonomous operation** |
| `calibration_demo.py` | 📏 Camera calibration only | Test distance measurement feature |
| `test_system.py` | 🧪 System testing | Verify all components work |

### 🚀 **For Normal Operation:**
```powershell
# Run the main autonomous robot
cd "c:\HARDARE\MicroLabGarbageCollector\Final-Model"
python enhanced_autonomous_robot.py
```

### 📏 **For Camera Calibration Only:**
```powershell
# Run calibration demo (simpler, calibration-focused)
cd "c:\HARDARE\MicroLabGarbageCollector\Final-Model"  
python calibration_demo.py
```

### 🧪 **For System Testing:**
```powershell
# Test all components
cd "c:\HARDARE\MicroLabGarbageCollector"
python Final-Model/test_system.py
```

---

# 🤖 Micro Lab Garbage Collector - Usage Guide

## 📁 **Which File to Run?**

### 🎯 **Main Project Files:**

| File | Purpose | When to Use |
|------|---------|-------------|
| **`enhanced_autonomous_robot.py`** | **🚀 MAIN ROBOT SYSTEM** | **Run this for full autonomous operation** |
| **`calibration_demo.py`** | **📏 Camera calibration tool** | **Run FIRST to calibrate distance measurement** |
| `test_system.py` | 🧪 System testing | Verify all components work |

### 📋 **Recommended Workflow:**

#### **Step 1: Calibrate Camera (First Time Only)**
```powershell
# Calibrate camera distance measurement
cd "c:\HARDARE\MicroLabGarbageCollector\Final-Model"
python calibration_demo.py
```

#### **Step 2: Run Main Robot**
```powershell
# Run the main autonomous robot (will use saved calibration)
cd "c:\HARDARE\MicroLabGarbageCollector\Final-Model"
python enhanced_autonomous_robot.py
```

### 🧪 **For System Testing:**
```powershell
# Test all components
cd "c:\HARDARE\MicroLabGarbageCollector"
python Final-Model/test_system.py
```

---

## 🎛️ **Controls & Usage**

### **📏 Calibration Tool** (`calibration_demo.py`)

**Purpose:** Create calibration data for camera distance measurement

#### **Camera Window Controls:**
- **'c'** - Toggle calibration mode on/off
- **'s'** - Save calibration sample (pauses for distance input)
- **'q'** - Quit and save calibration

#### **Calibration Process:**
1. Run: `python calibration_demo.py`
2. Camera window opens
3. Press **'c'** to enter calibration mode
4. Place 10×10 cm object in view (green bounding box appears)
5. Press **'s'** to save sample
6. ⏸️ **Tool pauses** - enter actual distance in terminal (e.g., "50")
7. Press Enter to save sample
8. 🔄 Repeat steps 4-7 for different distances (3-5 samples recommended)
9. Press **'q'** to finish and save calibration to `camera_calibration.json`

### **🚀 Main Robot System** (`enhanced_autonomous_robot.py`)

**Purpose:** Full autonomous garbage collection with distance measurement

#### **Camera Window Controls:**
- **'q'** - Quit application

#### **Features:**
- ✅ **Loads saved calibration** - Automatically uses `camera_calibration.json`
- ✅ **Real-time distance display** - Shows estimated distance to detected objects
- ✅ **Full autonomous operation** - Object detection, navigation, collection
- ✅ **Ultrasonic sensors** - Obstacle avoidance and approach distance
- ✅ **Hardware control** - ESP32 motor and servo commands
- ✅ **State machine** - Systematic search, object detection, pickup, sorting

---

## ⚙️ **Configuration**

### **Robot Settings:** `robot_config.env`
```bash
# Enable/disable camera distance measurement
ENABLE_CAMERA_DISTANCE=True

# Calibration object size (cm)
CALIBRATION_OBJECT_WIDTH=10.0
CALIBRATION_OBJECT_HEIGHT=10.0

# Distance measurement method
DISTANCE_MEASUREMENT_METHOD=average  # width, height, or average
```

### **Hardware Setup:**
- ESP32 connected to COM3 (configurable in robot_config.env)
- Camera (USB webcam index 0)
- Ultrasonic sensors on robot (front, left, right, rear)
- Servo motors for robotic arm
- DC motors for movement

---

## 🔧 **Troubleshooting**

### **Camera Issues:**
- Ensure USB camera is connected
- Check camera index in config (usually 0)
- Try different camera if issues persist

### **AI Model Issues:**
- Ensure `best.pt` file exists in root directory
- Install required packages: `pip install ultralytics torch`

### **Calibration Issues:**
- Use 10×10 cm object for best results
- Ensure good lighting and contrast
- Object should fill reasonable portion of camera view
- Try different distances for more accurate calibration

### **Serial Issues:**
- Check ESP32 connection on correct COM port
- Install ESP32 drivers if needed
- System works in mock mode if serial fails

---

## 📊 **System Status**

✅ **All 6 tests passing** - System fully operational  
✅ **Camera distance measurement** - Feature implemented and working  
✅ **Non-blocking calibration** - Camera continues during calibration input  
✅ **Complete autonomous operation** - Ready for garbage collection tasks

**Author:** Micro Lab Garbage Collector Team  
**Date:** September 2025  
**Version:** Enhanced Autonomous System with Distance Measurement