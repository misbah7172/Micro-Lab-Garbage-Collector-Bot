# Micro Lab Garbage Collector - Autonomous Robot System

##  System Overview

This is a comprehensive autonomous garbage collection robot system with advanced object detection, navigation, and classification capabilities. The robot can systematically search for objects, detect and approach them using computer vision, and pick/classify them without external localization systems like LIDAR.

##  System Architecture

### Hardware Components

#### Movement System
- **2x DC Motors**: Main propulsion with L298N motor driver
- **Motor Control**: PWM speed control with directional logic
- **Differential Drive**: Tank-style steering for precise movement

#### Sensing System
- **4x Ultrasonic Sensors (HC-SR04)**: 
  - Front: Object approach detection
  - Left/Right: Obstacle avoidance
  - Rear: Backup safety
- **1x Camera**: Object detection and visual guidance
- **1x DHT22**: Temperature and humidity monitoring
- **1x MQ-2**: Smoke/gas detection

#### Manipulation System
- **4x Servo Motors for Arm**:
  - 2x Servos: Arm lifting (shoulder + elbow)
  - 2x Servos: Gripper mechanism (open/close)
- **1x Servo Motor**: Bucket rotation for classification
- **Dual Bucket System**: Separate compartments for metal/non-metal objects

#### Control System
- **ESP32 Microcontroller**: Real-time hardware control
- **Python AI System**: Computer vision and decision making
- **Serial Communication**: Command interface between systems

##  Software Architecture

### State Machine Design

```
INITIALIZING → SEARCHING → OBJECT_DETECTED → APPROACHING → 
POSITIONING → PICKING → CLASSIFYING → DEPOSITING → SEARCHING
                     ↕
              AVOIDING_OBSTACLE
```

#### State Descriptions

1. **INITIALIZING**: System startup, move arm to home position
2. **SEARCHING**: Systematic area search with 360° rotations
3. **OBJECT_DETECTED**: Object found, begin alignment
4. **APPROACHING**: Move toward object while maintaining alignment
5. **POSITIONING**: Fine positioning before picking
6. **PICKING**: Execute pick sequence with arm
7. **CLASSIFYING**: Analyze object properties
8. **DEPOSITING**: Place object in appropriate bucket
9. **AVOIDING_OBSTACLE**: Emergency obstacle avoidance

### Threading Architecture

#### Main Threads
1. **Control Thread**: State machine execution (10 Hz)
2. **Vision Thread**: Camera processing and AI detection (30 Hz)
3. **Serial Thread**: ESP32 communication (2 Hz)

#### Thread Safety
- State machine protected with locks
- Sensor data shared via thread-safe data structures
- Command queue for serial communication

##  Navigation Without LIDAR

### Object Detection & Localization Strategy

#### Visual Detection
- **YOLO AI Model**: Real-time object detection
- **Bounding Box Analysis**: Object size and position estimation
- **Center Alignment**: Keep objects centered in camera view
- **Confidence Tracking**: Monitor detection stability over time

#### Distance Estimation
- **Ultrasonic Sensors**: Direct distance measurement to objects
- **Object Size Correlation**: Larger bounding boxes = closer objects
- **Approach Distance**: Stop at configurable distance (default 15cm)

#### Spatial Navigation
- **Spiral Search Pattern**: Systematic area coverage
- **Rotation-Based Search**: 360° rotation at each position
- **Obstacle Avoidance**: Reactive navigation using ultrasonic sensors
- **Direction Selection**: Choose path with maximum clearance

### Search Algorithm

```python
# Systematic Search Strategy
1. Rotate 360° at current position (2 complete rotations)
2. If object found → switch to OBJECT_DETECTED state
3. If no objects found → move to next search position
4. Repeat until area covered or object found
5. Return to starting position if no objects found
```

#### Search Pattern (Spiral)
```
Start → (0,0) → (1,0) → (1,1) → (0,1) → (-1,1) → 
(-1,0) → (-1,-1) → (0,-1) → (1,-1) → (2,-1) → ...
```

##  Object Tracking & Approach

### Visual Tracking
- **Object Center Calculation**: Track (x,y) position in camera frame
- **Alignment Threshold**: ±50 pixels from center (configurable)
- **Lost Object Recovery**: Return to search if object lost for >5 frames
- **History Tracking**: Maintain object position history for stability

### Approach Strategy
```python
# Approach Algorithm
while not at_target_distance:
    if object_lost:
        return_to_search()
    elif not_centered:
        rotate_to_align()
    elif obstacle_detected:
        avoid_obstacle()
    else:
        move_forward()
```

### Collision Avoidance
- **Real-time Monitoring**: Continuous ultrasonic sensor reading
- **Multi-directional Safety**: 4-corner obstacle detection
- **Emergency Stop**: Immediate stop if front distance < 5cm
- **Safe Direction Selection**: Move toward maximum clearance

##  Arm Control & Object Manipulation

### Pick Sequence
1. **Approach Position**: Extend arm toward object
2. **Gripper Open**: Prepare for grabbing
3. **Gripper Close**: Secure object
4. **Lift Up**: Raise object safely
5. **Classify Position**: Move to analysis position

### Classification System
- **AI-Based**: Use YOLO model class predictions
- **Metal Detection**: Configurable class IDs for metal objects
- **Confidence Threshold**: Minimum confidence for classification
- **Bucket Selection**: Route to appropriate compartment

### Deposit Sequence
1. **Bucket Positioning**: Rotate bucket to correct compartment
2. **Object Release**: Open gripper to drop object
3. **Statistics Update**: Track collection metrics
4. **Return Home**: Reset arm to home position

##  Configuration System

### Environment Variables (.env)
All system parameters configurable without code changes:

```bash
# Navigation
OBSTACLE_DISTANCE=20.0
OBJECT_APPROACH_DISTANCE=15.0
DETECTION_THRESHOLD=50

# AI Model
MODEL_PATH=best.pt
MODEL_CONFIDENCE_THRESHOLD=0.5

# Search Behavior
SEARCH_ROTATIONS=2
ROTATION_TIME=8.0

# Hardware
CAMERA_INDEX=0
SERIAL_PORT=AUTO
```

##  Usage Instructions

### Quick Start
```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Configure system (edit robot_config.env)
nano robot_config.env

# 3. Upload Arduino code to ESP32
# Upload ESP32_AutonomousController.ino

# 4. Run autonomous system
python enhanced_autonomous_robot.py
```

### Hardware Setup
1. **Connect Motors**: L298N driver to ESP32 PWM pins
2. **Mount Sensors**: 4 ultrasonics at robot corners
3. **Install Camera**: Front-facing for object detection
4. **Arm Assembly**: 4 servos in arm configuration
5. **Bucket System**: Rotating servo with dual compartments

### Software Configuration
1. **Model Training**: Train YOLO on your garbage types
2. **Class Configuration**: Set metal object class IDs
3. **Distance Calibration**: Adjust approach and obstacle distances
4. **Speed Tuning**: Set motor speeds for terrain
5. **Search Optimization**: Configure search pattern and timing

##  Monitoring & Statistics

### Real-time Display
- Current robot state
- Object detection count
- Sensor readings (distances, temperature, humidity)
- Collection statistics (total, metal, non-metal)
- Search progress

### Performance Metrics
- Objects collected per hour
- Classification accuracy
- Search efficiency
- Distance traveled
- Session uptime

##  Troubleshooting

### Common Issues

#### No Objects Detected
- Check camera connection and lighting
- Verify AI model path and loading
- Adjust confidence threshold
- Ensure objects are in model training data

#### Navigation Problems
- Calibrate ultrasonic sensors
- Check motor connections and power
- Verify ESP32 serial communication
- Adjust obstacle distance thresholds

#### Picking Failures
- Check servo power and connections
- Calibrate arm positions
- Verify gripper mechanism
- Adjust approach distance

#### Communication Errors
- Verify serial port and baud rate
- Check ESP32 power and programming
- Test cable connections
- Enable verbose output for debugging

##  Advanced Features

### Adaptive Search
- Dynamic search pattern based on success rate
- Learning optimal search positions
- Adaptive rotation timing

### Enhanced Classification
- Multi-modal classification (visual + weight)
- Uncertainty handling
- Classification confidence reporting

### Fault Recovery
- Automatic error recovery
- State restoration after failures
- Safe mode operation

### Performance Optimization
- Intelligent power management
- Predictive navigation
- Optimized search patterns

##  Future Enhancements

### Navigation Improvements
- SLAM implementation with visual odometry
- Path planning with obstacle mapping
- Multi-robot coordination

### Advanced Manipulation
- Force feedback for gripper
- Object orientation detection
- Delicate object handling

### Intelligence Upgrades
- Reinforcement learning for navigation
- Adaptive behavior based on environment
- Predictive object location

##  Hardware Shopping List

### Essential Components
- ESP32 Development Board
- 2x DC Geared Motors (12V)
- L298N Motor Driver
- 4x HC-SR04 Ultrasonic Sensors
- 4x Servo Motors (MG996R or similar)
- 1x Servo Motor for bucket (smaller)
- USB Camera or ESP32-CAM
- DHT22 Temperature/Humidity Sensor
- MQ-2 Gas/Smoke Sensor
- 12V Battery Pack
- Jumper wires and breadboard
- Chassis/frame materials
- Wheels and casters

### Optional Upgrades
- IMU (MPU6050) for orientation
- Encoders for precise movement
- RGB LED indicators
- Buzzer for audio feedback
- SD card for data logging

##  Learning Resources

### Computer Vision
- YOLO object detection training
- OpenCV basics and advanced techniques
- Camera calibration and perspective

### Robotics
- Differential drive kinematics
- PID control for navigation
- State machine design patterns

### Electronics
- Motor control and PWM
- Sensor interfacing
- Power management for robots

This system provides a complete solution for autonomous garbage collection without expensive sensors like LIDAR, using clever combinations of computer vision, ultrasonic sensing, and intelligent algorithms.