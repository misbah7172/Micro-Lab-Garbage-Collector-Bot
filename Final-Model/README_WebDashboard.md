# 🌐 Micro Lab Garbage Collector - Web Dashboard

## Overview
This is a modern, browser-based dashboard for the AI-powered garbage collection robot. The dashboard provides real-time monitoring and control through a clean, responsive web interface.

## 🚀 Quick Start

### 1. Install Dependencies
```bash
cd Final-Model
pip install -r requirements.txt
```

### 2. Run the Web Dashboard
```bash
python web_dashboard.py
```

### 3. Open Dashboard
Navigate to: **http://localhost:5000**

## 📊 Dashboard Features

### 🎥 **Camera Feed Panel**
- **Live video stream** from the robot's camera in a dedicated panel
- **Real-time AI detection** with bounding boxes around detected objects
- **Performance overlay** showing FPS and detection count
- **Camera status indicator** (Online/Offline)

### ⚙️ **System Status Panel**
- **Robot Status**: Current operational state (READY, MOVING, COLLECTING, etc.)
- **Movement Direction**: Current movement (FORWARD, ROTATING, STOPPED, COLLECTING)
- **ESP32 Connection**: Serial connection status with microcontroller
- **AI Model Status**: YOLO model loading and operational status
- **System Uptime**: How long the system has been running
- **Last Command**: Most recent command sent to robot

### 🎯 **Detection & Collection Panel**
- **Live Detections**: Current number of objects detected
- **Items Collected**: Total count of successfully collected items
- **Collection Progress**: Whether collection is currently in progress
- **Last Collection Time**: Timestamp of most recent collection

### 🌡️ **Environmental Sensors Panel**
- **Temperature**: Current ambient temperature from DHT22 sensor
- **Humidity**: Current humidity percentage from DHT22 sensor
- **Smoke Level**: Gas/smoke detection level from MQ-2 sensor
- **Distance**: Obstacle distance from ultrasonic sensor

### 🎮 **Robot Control Panel**
- **Forward**: Move robot forward
- **Rotate**: Rotate robot to search for objects
- **Stop**: Emergency stop all movement
- **Collect**: Trigger collection sequence
- **Heartbeat**: Send keepalive signal

### 📈 **Performance Metrics Panel**
- **FPS**: Real-time frames per second of camera processing
- **Detection Rate**: Efficiency percentage of AI detection
- **System Load**: Visual progress bar of system resource usage
- **Memory Usage**: Visual progress bar of memory consumption

## 🎨 Design Features

### **Modern UI Elements**
- **Glass-morphism design** with backdrop blur effects
- **Gradient backgrounds** and smooth animations
- **Responsive layout** that adapts to different screen sizes
- **Color-coded status indicators** for quick visual feedback
- **Font Awesome icons** for professional appearance

### **Real-time Updates**
- **WebSocket communication** for instant data updates
- **Live camera streaming** with base64 encoding
- **Status synchronization** between robot and dashboard
- **Smooth transitions** and loading animations

### **Interactive Controls**
- **Click-to-command** buttons with visual feedback
- **Keyboard shortcuts** for quick robot control:
  - `W` or `↑`: Forward
  - `R`: Rotate
  - `S` or `↓`: Stop
  - `C` or `Space`: Collect
  - `H`: Heartbeat

## 🔧 Technical Architecture

### **Backend (Flask + SocketIO)**
- **Flask web server** hosting the dashboard interface
- **SocketIO** for real-time bidirectional communication
- **Threading** for concurrent camera processing and serial communication
- **OpenCV** for camera capture and image processing
- **YOLO AI model** for object detection
- **Serial communication** with ESP32 microcontroller

### **Frontend (HTML + CSS + JavaScript)**
- **Responsive CSS Grid** layout for panel organization
- **Socket.IO client** for real-time data synchronization
- **Base64 image streaming** for live camera feed
- **Modern CSS animations** and visual effects
- **Cross-browser compatibility** with modern web standards

### **Communication Protocol**
```
Web Browser ←→ Flask Server ←→ ESP32 Robot
     ↑              ↑              ↑
 Dashboard      SocketIO      Serial USB
```

## 📱 Mobile Responsiveness

The dashboard automatically adapts to different screen sizes:
- **Desktop**: Full 3-column layout with all panels visible
- **Tablet**: 2-column layout with camera panel spanning full width
- **Mobile**: Single column stacked layout for optimal mobile viewing

## 🔒 Security Features

- **PyTorch model loading** with trusted source handling
- **CORS protection** for cross-origin requests
- **Input validation** for robot commands
- **Error handling** and graceful degradation

## 🚨 Troubleshooting

### Camera Issues
- Ensure camera is connected and not being used by other applications
- Check camera permissions in Windows settings
- Try changing camera index in code if multiple cameras are connected

### ESP32 Connection Issues
- Verify ESP32 is connected via USB
- Check COM port assignment in Device Manager
- Ensure ESP32 drivers are installed (CH340 or CP210x)

### Model Loading Issues
- Verify `best.pt` file exists in the Final-Model directory
- Check PyTorch installation and compatibility
- Ensure sufficient system memory for model loading

### Network Issues
- Verify port 5000 is not blocked by firewall
- Check if another application is using port 5000
- Try accessing via `127.0.0.1:5000` instead of `localhost:5000`

## 🎯 Usage Tips

1. **Keep the dashboard open** while operating the robot for real-time monitoring
2. **Use keyboard shortcuts** for quick robot control during operation
3. **Monitor environmental sensors** to ensure safe operating conditions
4. **Watch the camera feed** to verify AI detection accuracy
5. **Check connection status** regularly to ensure stable communication

## 🔄 System Requirements

- **Python 3.8+** with required packages installed
- **USB camera** (built-in webcam or external)
- **ESP32 microcontroller** with garbage collector firmware
- **Modern web browser** (Chrome, Firefox, Safari, Edge)
- **Windows 10/11** (tested environment)

## 📞 Support

For technical support or questions about the web dashboard:
1. Check the terminal output for error messages
2. Verify all dependencies are correctly installed
3. Ensure hardware connections are secure
4. Review the troubleshooting section above

---

**🤖 Micro Lab Garbage Collector - Making the world cleaner with AI!**