# 🔧 Environment Configuration Guide

## Overview
The `.env` file contains all configurable parameters for the Micro Lab Garbage Collector system. This allows you to easily change settings like model paths, camera settings, and serial ports without modifying the code.

## 📁 Quick Model Switching

### Using the Configuration Manager (Recommended)
```bash
python config_manager.py
```
This interactive tool helps you:
- 🔍 List all available models
- 🔄 Switch between different models
- 🎛️ Enable/disable mock modes
- 📋 View current configuration
- 🔧 Reset to defaults

### Manual .env Editing
Simply edit the `.env` file and change the `MODEL_PATH` setting:

```env
# Different model examples:
MODEL_PATH=best.pt                          # Local model in same directory
MODEL_PATH=models/custom_model.pt           # Model in subdirectory
MODEL_PATH=../Test-Model/best.pt           # Model in parent directory
MODEL_PATH=C:/Models/experimental.pt        # Absolute path
MODEL_PATH=yolov8n.pt                      # Download from Ultralytics
```

## ⚙️ Configuration Categories

### 🤖 AI Model Settings
```env
MODEL_PATH=best.pt                    # Path to YOLO model file
MODEL_INPUT_SIZE=480                  # Input resolution for AI processing
MODEL_CONFIDENCE_THRESHOLD=0.5        # Detection confidence threshold
BOTTLE_CLASS_ID=39                    # COCO class ID for bottles
```

### 📹 Camera Configuration
```env
CAMERA_INDEX=0                        # Camera device index (0, 1, 2...)
CAMERA_WIDTH=640                      # Camera resolution width
CAMERA_HEIGHT=480                     # Camera resolution height
CAMERA_FPS=30                         # Target frames per second
JPEG_QUALITY=85                       # Compression quality (1-100)
```

### 🔌 Serial Communication
```env
SERIAL_PORT=AUTO                      # AUTO for auto-detection or specific port
SERIAL_BAUD_RATE=115200              # Communication speed
SERIAL_TIMEOUT=1.0                   # Read timeout in seconds
AUTO_DETECT_PORTS=CH340,CP210,USB   # Device identifiers for auto-detection
```

### 🌐 Web Server Settings
```env
WEB_HOST=0.0.0.0                     # Server host (0.0.0.0 for all interfaces)
WEB_PORT=5000                        # Server port
WEB_DEBUG=False                      # Enable Flask debug mode
SECRET_KEY=micro_lab_garbage_collector_2025  # Session security key
```

### 🎮 Robot Commands
```env
COMMAND_FORWARD=F                    # Move forward command
COMMAND_ROTATE=R                     # Rotate command
COMMAND_STOP=S                       # Stop command
COMMAND_COLLECT=C                    # Collect trash command
COMMAND_HEARTBEAT=H                  # Heartbeat/ping command
```

### 🔧 Development & Testing
```env
MOCK_ESP32=False                     # Simulate ESP32 without hardware
MOCK_CAMERA=False                    # Simulate camera without hardware
VERBOSE_OUTPUT=True                  # Enable detailed console output
```

## 🚀 Common Use Cases

### 🔄 Switching Between Models
1. **Testing different YOLO versions:**
   ```env
   MODEL_PATH=yolov8n.pt    # Fastest, least accurate
   MODEL_PATH=yolov8s.pt    # Small
   MODEL_PATH=yolov8m.pt    # Medium
   MODEL_PATH=yolov8l.pt    # Large
   MODEL_PATH=yolov8x.pt    # Extra large, most accurate
   ```

2. **Using custom trained models:**
   ```env
   MODEL_PATH=models/garbage_v1.pt
   MODEL_PATH=models/indoor_detection.pt
   MODEL_PATH=../experiments/latest_model.pt
   ```

### 📹 Camera Troubleshooting
1. **Multiple cameras connected:**
   ```env
   CAMERA_INDEX=0    # Built-in webcam
   CAMERA_INDEX=1    # External USB camera
   CAMERA_INDEX=2    # Second external camera
   ```

2. **Performance optimization:**
   ```env
   # Lower resolution for better performance
   CAMERA_WIDTH=320
   CAMERA_HEIGHT=240
   CAMERA_FPS=15
   
   # Higher quality for better detection
   CAMERA_WIDTH=1280
   CAMERA_HEIGHT=720
   JPEG_QUALITY=95
   ```

### 🔌 Serial Port Configuration
1. **Windows systems:**
   ```env
   SERIAL_PORT=COM3
   SERIAL_PORT=COM4
   SERIAL_PORT=AUTO    # Recommended
   ```

2. **Linux/Mac systems:**
   ```env
   SERIAL_PORT=/dev/ttyUSB0
   SERIAL_PORT=/dev/ttyACM0
   SERIAL_PORT=/dev/cu.usbserial-*
   ```

### 🧪 Development & Testing
1. **Hardware-free testing:**
   ```env
   MOCK_ESP32=True      # No ESP32 required
   MOCK_CAMERA=True     # No camera required
   VERBOSE_OUTPUT=True  # Detailed logging
   ```

2. **Remote deployment:**
   ```env
   WEB_HOST=0.0.0.0     # Accept connections from any IP
   WEB_PORT=8080        # Custom port
   WEB_DEBUG=False      # Production mode
   ```

## 🛠️ Advanced Configuration

### 🎯 Performance Tuning
```env
TARGET_FPS=30                        # Target processing speed
FRAME_PROCESSING_DELAY=0.033         # Delay between frames (1/30 = 0.033)
MODEL_INPUT_SIZE=480                 # Lower = faster, higher = more accurate
```

### 📊 Detection Settings
```env
MODEL_CONFIDENCE_THRESHOLD=0.3       # Lower = more detections (more false positives)
MODEL_CONFIDENCE_THRESHOLD=0.7       # Higher = fewer detections (more accurate)
BOTTLE_CLASS_ID=39                   # COCO dataset bottle class
```

### 🔐 Security & Networking
```env
SECRET_KEY=your_unique_secret_key    # Change for production
WEB_HOST=127.0.0.1                  # Local only
WEB_HOST=0.0.0.0                    # Accept external connections
```

## 📋 Configuration Validation

The system automatically validates configuration on startup:
- ✅ Checks if model file exists
- ✅ Validates camera index
- ✅ Tests serial port connectivity
- ✅ Verifies web server settings

## 🚨 Troubleshooting

### Model Not Found
```
Error: Model file not found at: best.pt
```
**Solution:** Update `MODEL_PATH` in `.env` or use `config_manager.py`

### Camera Issues
```
Error: Failed to open camera
```
**Solutions:**
- Try different `CAMERA_INDEX` values (0, 1, 2...)
- Enable `MOCK_CAMERA=True` for testing
- Check camera permissions

### Serial Connection Problems
```
Error: Could not connect to ESP32
```
**Solutions:**
- Set `SERIAL_PORT=AUTO` for auto-detection
- Try specific ports like `COM3`, `COM4`
- Enable `MOCK_ESP32=True` for software-only testing

### Port Already in Use
```
Error: Port 5000 is already in use
```
**Solution:** Change `WEB_PORT` to a different value (e.g., 8080)

## 💡 Tips & Best Practices

1. **Keep backups:** Copy `.env` to `.env.backup` before major changes
2. **Use mock modes:** Test software changes without hardware
3. **Version control:** Add `.env.example` to git, but keep `.env` private
4. **Documentation:** Comment your custom settings in the `.env` file
5. **Validation:** Use `config_manager.py` to verify settings

## 🔄 Environment File Templates

### Production Template
```env
MODEL_PATH=best.pt
CAMERA_INDEX=0
SERIAL_PORT=AUTO
WEB_HOST=0.0.0.0
WEB_PORT=5000
MOCK_ESP32=False
MOCK_CAMERA=False
VERBOSE_OUTPUT=False
```

### Development Template
```env
MODEL_PATH=yolov8n.pt
CAMERA_INDEX=0
SERIAL_PORT=AUTO
WEB_HOST=127.0.0.1
WEB_PORT=5000
MOCK_ESP32=True
MOCK_CAMERA=False
VERBOSE_OUTPUT=True
```

### Testing Template
```env
MODEL_PATH=best.pt
CAMERA_INDEX=0
SERIAL_PORT=AUTO
WEB_HOST=127.0.0.1
WEB_PORT=8080
MOCK_ESP32=True
MOCK_CAMERA=True
VERBOSE_OUTPUT=True
```

---

**🔧 Need help?** Run `python config_manager.py` for an interactive configuration tool!