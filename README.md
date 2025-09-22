#  Micro Lab Garbage Collector

An AI-powered autonomous garbage collection robot using ESP32, computer vision, and environmental monitoring.

##  Features

- **AI-based Object Detection**: Utilizes YOLO with a laptop webcam for real-time trash detection
- **Autonomous Operation**: Runs without manual intervention using ESP32 microcontroller
- **Environmental Monitoring**: Tracks temperature, humidity, and smoke levels
- **Remote Access**: Controlled and monitored via the Blynk IoT app
- **Failsafe Mechanism**: Automatically stops if communication is lost
- **Robotic Arm**: Picks up and disposes of detected trash automatically

##  Functionalities

1. **Trash Detection**: Identifies trash using AI-powered computer vision
2. **Navigation**: Moves autonomously towards detected objects with ESP32 control
3. **Obstacle Avoidance**: Uses ultrasonic sensors for safe movement
4. **Trash Collection**: Picks up trash using a 3-axis robotic arm with servo motors
5. **Trash Disposal**: Places collected trash into a designated bin
6. **Sensor Data Reporting**: Sends environmental data to the Blynk app
7. **Real-time Monitoring**: Live video feed with detection overlays


### Data Flow
1. **Laptop camera** captures live video feed
2. **Python script** processes frames using YOLO AI model
3. **Detection results** sent to ESP32 via USB serial connection
4. **ESP32 controls** robot movement, arm, and sensors
5. **Environmental data** sent to Blynk app for monitoring
6. **Real-time feedback** displayed on laptop screen

### Communication Protocol
- **Laptop → ESP32**: Movement commands (`F`, `R`, `S`, `C`, `H`)
- **ESP32 → Blynk**: Sensor data and status updates
- **Blynk → ESP32**: Remote control commands

##  Wiring Diagram

### ESP32 Pin Connections

#### Motor Driver (L298N)
```
ESP32 Pin    →    L298N Pin
GPIO 2       →    IN1 (Left Motor Forward)
GPIO 4       →    IN2 (Left Motor Backward)
GPIO 16      →    IN3 (Right Motor Forward)
GPIO 17      →    IN4 (Right Motor Backward)
GPIO 5       →    ENA (Left Motor Enable)
GPIO 18      →    ENB (Right Motor Enable)
```

#### Sensors
```
ESP32 Pin    →    Sensor
GPIO 12      →    HC-SR04 TRIG
GPIO 14      →    HC-SR04 ECHO
GPIO 23      →    DHT22 Data
GPIO 35      →    MQ-2 Analog Out
```

#### Servo Motors (Robotic Arm)
```
ESP32 Pin    →    Servo
GPIO 25      →    Base Servo Signal
GPIO 26      →    Arm Servo Signal
GPIO 27      →    Gripper Servo Signal
```

#### Power and Ground
```
ESP32 VIN    →    L298N +12V (Battery Positive)
ESP32 GND    →    Common Ground
ESP32 3.3V   →    Sensors VCC
```

##  Software Setup

### 1. Arduino IDE Setup

1. **Install Arduino IDE** (version 1.8.x or 2.x)
2. **Add ESP32 Board Support**:
   - Go to File → Preferences
   - Add to Additional Board Manager URLs: 
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Go to Tools → Board → Boards Manager
   - Search "ESP32" and install "ESP32 by Espressif Systems"

3. **Install Required Libraries**:
   ```
   - Blynk (for IoT connectivity)
   - DHT sensor library (for temperature/humidity)
   - ESP32Servo (for servo motor control)
   ```

### 2. Python Environment Setup

1. **Install Python 3.8+** from [python.org](https://python.org)

2. **Install required packages**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Verify installations**:
   ```bash
   python -c "import cv2, ultralytics, serial, supervision; print('All packages installed successfully')"
   ```

### 3. Blynk Setup

1. **Download Blynk App** (iOS/Android)
2. **Create New Project**:
   - Choose ESP32 as device
   - Copy the Auth Token
3. **Add Widgets**:
   - V0: Button (System ON/OFF)
   - V1: Value Display (Temperature)
   - V2: Value Display (Humidity)
   - V3: Value Display (Smoke Level)
   - V4: Value Display (Distance)
   - V5: Value Display (System Status)
   - V6: Value Display (Robot State)
   - V7: Button (Reset Arm)

## 🛠️ Hardware Requirements

### Laptop/Computer (AI Processing Station)
| Component | Specification | Purpose |
|-----------|---------------|---------|
| **Laptop/Desktop** | Windows 10/11, 8GB+ RAM | AI processing and control |
| **Built-in/USB Webcam** | 720p or higher resolution | Live video feed for detection |
| **USB Port** | USB 2.0/3.0 | Serial connection to ESP32 |
| **Python 3.8+** | Latest version recommended | Running detection script |

### Connection Overview
- **Laptop ↔ ESP32**: USB cable for serial communication
- **ESP32**: Controls all robot hardware (motors, sensors, arm)
- **Power**: Independent battery system for robot mobility

##  Installation Steps

### Step 1: Hardware Assembly

1. **Build the chassis**:
   - Mount motors to the chassis
   - Attach wheels to motors
   - Install battery holder

2. **Mount ESP32 and components**:
   - Secure ESP32 to chassis
   - Mount L298N motor driver
   - Install sensors in appropriate locations

3. **Assemble robotic arm**:
   - Mount base servo to chassis
   - Attach arm servo to base
   - Install gripper servo at end
   - Create simple gripper mechanism

4. **Complete wiring** according to the diagram above

5. **Power connections**:
   - Connect 18650 batteries in series (7.4V)
   - Add power switch for safety
   - Ensure proper voltage regulation

### Step 2: Software Configuration

1. **Configure ESP32 code**:
   ```cpp
   // Update these in MicroLabGarbageCollector.ino
   #define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```

2. **Upload ESP32 code**:
   - Select "ESP32 Dev Module" as board
   - Choose correct COM port
   - Upload the code

3. **Configure Python script**:
   - Ensure `best.pt` YOLO model is in the project folder
   - The script will auto-detect the ESP32 COM port
   - If auto-detection fails, manually set the port in the code

### Step 3: YOLO Model Setup

1. **Train your model** (or use pre-trained):
   - Use YOLOv8 for garbage detection
   - Train on garbage/trash dataset
   - Export as `best.pt`

2. **Place model file**:
   - Copy `best.pt` to project directory
   - Ensure path is correct in `test.py`

##  Usage Instructions

### Pre-Operation Setup

1. **Connect hardware**:
   - Connect ESP32 to laptop via USB cable
   - Ensure robot is powered with batteries
   - Check ESP32 connection in Device Manager (should show COM port)

2. **Position laptop**:
   - Place laptop where camera has clear view of operating area
   - Ensure laptop is plugged in or has sufficient battery
   - Position for optimal WiFi signal (for Blynk connectivity)

### Starting the System

1. **Power on the robot**:
   - Turn on robot battery power switch
   - ESP32 should connect to WiFi (LED indicator)
   - Check ESP32 serial connection to laptop

2. **Start Python detection on laptop**:
   ```bash
   cd C:\HARDARE\MicroLabGarbageCollector
   python test.py
   ```
   - Python script will auto-detect ESP32 COM port
   - Camera window should open showing live feed
   - Wait for "System initialized successfully!" message

3. **Verify Blynk connection**:
   - Open Blynk app on phone
   - Check system status (should show "ONLINE")
   - Environmental sensors should display data

4. **Activate autonomous mode**:
   - Press System ON button in Blynk app, OR
   - System automatically starts when Python script detects trash
   - Robot should begin searching for garbage

### Operation Workflow

1. **Detection Phase**: 
   - Laptop camera continuously scans environment
   - YOLO AI identifies trash objects in real-time
   - Detection results shown in laptop video window

2. **Command Phase**:
   - Python script sends movement commands to ESP32 via USB
   - Commands: `F` (forward), `R` (rotate), `S` (stop), `C` (collect)

3. **Robot Response**:
   - ESP32 receives commands and controls robot hardware
   - Motors move robot toward detected trash
   - Ultrasonic sensor prevents collisions
   - Robotic arm activates for collection

4. **Monitoring Phase**:
   - Environmental data sent to Blynk app
   - Real-time robot status available on phone
   - Live video feed on laptop shows progress

### Operation Modes

- **Search Mode**: Robot rotates to scan for garbage
- **Approach Mode**: Moves toward detected trash
- **Collection Mode**: Uses robotic arm to pick up trash
- **Disposal Mode**: Moves to disposal area and drops trash

### Manual Controls

**Keyboard Controls** (when Python script is active):
- `ESC`: Emergency stop and exit
- `S`: Stop robot movement
- `C`: Force trash collection sequence

**Blynk App Controls**:
- System ON/OFF button
- Reset Arm button
- Real-time sensor monitoring

##  Monitoring

### Environmental Data
- **Temperature**: Real-time monitoring with alerts
- **Humidity**: Environmental conditions
- **Smoke Level**: Fire/smoke detection
- **Distance**: Obstacle detection range

### System Status
- **Connection Status**: Online/Offline
- **Robot State**: Searching/Moving/Collecting
- **Detection Count**: Number of items detected
- **Frame Rate**: Video processing speed

##  Troubleshooting

### Common Issues

1. **ESP32 won't connect to WiFi**:
   - Check SSID and password
   - Ensure WiFi is 2.4GHz
   - Check signal strength

2. **Serial communication fails**:
   - Verify COM port in Device Manager
   - Check USB cable connection
   - Restart both devices

3. **Camera not detected**:
   - Close other applications using camera
   - Check camera permissions
   - Try different USB port

4. **Motors not responding**:
   - Check power supply voltage
   - Verify L298N connections
   - Test with multimeter

5. **Servo arm not moving**:
   - Check servo power supply
   - Verify PWM connections
   - Test individual servos

### Debug Commands

**ESP32 Serial Monitor**:
```
F - Move forward
R - Rotate right
S - Stop
C - Collect trash
H - Heartbeat
```

**Python Debug**:
```bash
# Test camera only
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error')"

# Test serial connection
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"
```

##  Customization

### Modifying Detection Classes
Edit the YOLO model training to detect specific types of garbage:
- Plastic bottles
- Cans
- Paper waste
- Organic waste

### Adjusting Movement Parameters
In `MicroLabGarbageCollector.ino`:
```cpp
// Motor speed (0-255)
analogWrite(MOTOR_LEFT_ENABLE, 200);

// Detection distance threshold
if (getDistance() > 20) {
```

### Adding New Sensors
1. Define new pins in ESP32 code
2. Add sensor reading functions
3. Update Blynk virtual pins
4. Modify monitoring dashboard

##  Maintenance

### Regular Checks
- Battery voltage levels
- Motor brush condition
- Sensor calibration
- Servo arm alignment
- WiFi connection stability

### Cleaning
- Clean camera lens regularly
- Remove debris from wheels
- Check for loose connections
- Update software as needed

##  Contributing

1. Fork the repository
2. Create feature branch
3. Commit changes
4. Push to branch
5. Create Pull Request

##  License

This project is licensed under the MIT License - see the LICENSE file for details.

##  Support

For issues and questions:
- Create an issue on GitHub
- Check the troubleshooting section
- Review hardware connections
- Verify software dependencies

##  Future Enhancements

- [ ] GPS navigation for larger areas
- [ ] Multi-camera setup for 360° vision
- [ ] AI-powered path planning
- [ ] Trash sorting capabilities
- [ ] Solar panel charging
- [ ] Mobile app for direct control
- [ ] Fleet management for multiple robots

---

**Happy Building!**

Remember to test each component individually before integrating the complete system.