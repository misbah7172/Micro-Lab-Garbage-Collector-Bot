#!/usr/bin/env python3
"""
Micro Lab Garbage Collector - Enhanced Autonomous Robot System
With configuration file support and improved object detection/localization
"""

import os
import cv2
import serial
import serial.tools.list_ports
import supervision as sv
from ultralytics import YOLO
import warnings
import time
import threading
import queue
import math
import random
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List
import numpy as np
from dotenv import load_dotenv

# Load configuration
load_dotenv('robot_config.env')
warnings.filterwarnings("ignore", category=FutureWarning)

class RobotState(Enum):
    """Robot operational states"""
    INITIALIZING = "initializing"
    SEARCHING = "searching"
    OBJECT_DETECTED = "object_detected"
    APPROACHING = "approaching"
    POSITIONING = "positioning"
    PICKING = "picking"
    CLASSIFYING = "classifying"
    DEPOSITING = "depositing"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    RETURNING_HOME = "returning_home"
    ERROR = "error"

class MovementDirection(Enum):
    """Movement directions"""
    FORWARD = "F"
    BACKWARD = "B"
    LEFT = "L"
    RIGHT = "R"
    ROTATE_LEFT = "RL"
    ROTATE_RIGHT = "RR"
    STOP = "S"

class ArmPosition(Enum):
    """Arm positions for picking mechanism"""
    HOME = "A0"
    APPROACH = "A1"
    GRAB_OPEN = "A2"
    GRAB_CLOSE = "A3"
    LIFT_UP = "A4"
    CLASSIFY = "A5"

class BucketPosition(Enum):
    """Bucket positions for object classification"""
    NEUTRAL = "B0"
    METAL = "B1"
    NON_METAL = "B2"

@dataclass
class SensorData:
    """Sensor readings from ESP32"""
    front_distance: float = 999.0
    left_distance: float = 999.0
    right_distance: float = 999.0
    rear_distance: float = 999.0
    temperature: float = 25.0
    humidity: float = 60.0
    smoke_level: int = 0

@dataclass
class DetectionData:
    """Object detection information"""
    object_count: int = 0
    object_center_x: int = 0
    object_center_y: int = 0
    object_confidence: float = 0.0
    object_class: str = "unknown"
    object_class_id: int = -1
    is_metal: bool = False
    bbox_area: float = 0.0

class RobotConfig:
    """Configuration class loaded from environment"""
    
    def __init__(self):
        # AI Model Configuration
        self.MODEL_PATH = os.getenv('MODEL_PATH', 'best.pt')
        self.MODEL_INPUT_SIZE = int(os.getenv('MODEL_INPUT_SIZE', '480'))
        self.MODEL_CONFIDENCE_THRESHOLD = float(os.getenv('MODEL_CONFIDENCE_THRESHOLD', '0.5'))
        
        # Camera Configuration
        self.CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
        self.CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '640'))
        self.CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '480'))
        self.CAMERA_FPS = int(os.getenv('CAMERA_FPS', '30'))
        
        # Serial Communication
        self.SERIAL_PORT = os.getenv('SERIAL_PORT', 'AUTO')
        self.SERIAL_BAUD_RATE = int(os.getenv('SERIAL_BAUD_RATE', '115200'))
        self.SERIAL_TIMEOUT = float(os.getenv('SERIAL_TIMEOUT', '1.0'))
        
        # Navigation Parameters
        self.OBSTACLE_DISTANCE = float(os.getenv('OBSTACLE_DISTANCE', '20.0'))
        self.OBJECT_APPROACH_DISTANCE = float(os.getenv('OBJECT_APPROACH_DISTANCE', '15.0'))
        self.MOVEMENT_DURATION = float(os.getenv('MOVEMENT_DURATION', '2.0'))
        self.DETECTION_THRESHOLD = int(os.getenv('DETECTION_THRESHOLD', '50'))
        
        # Search Behavior
        self.SEARCH_ROTATIONS = int(os.getenv('SEARCH_ROTATIONS', '2'))
        self.ROTATION_TIME = float(os.getenv('ROTATION_TIME', '8.0'))
        
        # Classification
        metal_ids = os.getenv('METAL_CLASS_IDS', '0,1,2')
        self.METAL_CLASS_IDS = [int(x.strip()) for x in metal_ids.split(',')]
        self.CONFIDENCE_THRESHOLD = float(os.getenv('CONFIDENCE_THRESHOLD', '0.6'))
        
        # Safety Settings
        self.MAX_SEARCH_POSITIONS = int(os.getenv('MAX_SEARCH_POSITIONS', '10'))
        self.EMERGENCY_STOP_DISTANCE = float(os.getenv('EMERGENCY_STOP_DISTANCE', '5.0'))
        
        # Debug Settings
        self.VERBOSE_OUTPUT = os.getenv('VERBOSE_OUTPUT', 'True').lower() == 'true'
        self.SHOW_CAMERA_FEED = os.getenv('SHOW_CAMERA_FEED', 'True').lower() == 'true'
        self.LOG_STATE_TRANSITIONS = os.getenv('LOG_STATE_TRANSITIONS', 'True').lower() == 'true'
        
        # Performance Settings
        self.CONTROL_LOOP_FREQUENCY = int(os.getenv('CONTROL_LOOP_FREQUENCY', '10'))
        self.VISION_LOOP_FREQUENCY = int(os.getenv('VISION_LOOP_FREQUENCY', '30'))

class EnhancedAutonomousRobot:
    """Enhanced autonomous robot with improved object detection and localization"""
    
    def __init__(self):
        self.config = RobotConfig()
        
        # System components
        self.serial_connection: Optional[serial.Serial] = None
        self.camera: Optional[cv2.VideoCapture] = None
        self.ai_model: Optional[YOLO] = None
        
        # Threading components
        self.running = False
        self.sensor_data = SensorData()
        self.detection_data = DetectionData()
        
        # Robot state management
        self.current_state = RobotState.INITIALIZING
        self.previous_state = RobotState.INITIALIZING
        self.state_start_time = time.time()
        self.state_machine_lock = threading.Lock()
        
        # Navigation and search variables
        self.search_rotation_count = 0
        self.search_position_count = 0
        self.start_position = (0, 0)  # Virtual coordinates
        self.current_position = (0, 0)
        self.search_pattern = self.generate_search_pattern()
        
        # Object tracking and localization
        self.target_object_history = []  # Track object positions over time
        self.object_lost_count = 0
        self.max_object_lost = 5
        
        # Performance metrics
        self.frame_center_x = self.config.CAMERA_WIDTH // 2
        self.frame_center_y = self.config.CAMERA_HEIGHT // 2
        
        # Statistics
        self.objects_collected = 0
        self.metal_objects = 0
        self.non_metal_objects = 0
        self.total_distance_traveled = 0
        self.session_start_time = time.time()
        
        # Initialize annotators
        self.setup_annotators()

    def setup_annotators(self):
        """Setup supervision annotators"""
        self.box_annotator = sv.BoxAnnotator(
            color=sv.ColorPalette.default(),
            thickness=2,
            text_thickness=1,
            text_scale=0.5
        )
        self.label_annotator = sv.LabelAnnotator(
            color=sv.ColorPalette.default(),
            text_thickness=1,
            text_scale=0.5
        )

    def generate_search_pattern(self) -> List[Tuple[int, int]]:
        """Generate spiral search pattern for systematic area coverage"""
        pattern = []
        x, y = 0, 0
        dx, dy = 1, 0
        steps = 1
        
        for _ in range(self.config.MAX_SEARCH_POSITIONS):
            pattern.append((x, y))
            
            # Move in current direction
            x += dx
            y += dy
            steps -= 1
            
            # Change direction when steps reach 0
            if steps == 0:
                dx, dy = -dy, dx  # Rotate 90 degrees
                if dx == 0:  # Completed a full cycle
                    steps = abs(y) * 2 + 1
                else:
                    steps = abs(x) * 2
        
        return pattern

    def find_esp32_port(self) -> Optional[str]:
        """Automatically find ESP32 port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if any(keyword in port.description for keyword in ["CH340", "CP210", "USB"]):
                return port.device
        return None

    def setup_serial_connection(self) -> bool:
        """Setup serial connection with ESP32"""
        try:
            if self.config.SERIAL_PORT == "AUTO":
                auto_port = self.find_esp32_port()
                port = auto_port if auto_port else "COM3"
            else:
                port = self.config.SERIAL_PORT
            
            self.serial_connection = serial.Serial(
                port, 
                self.config.SERIAL_BAUD_RATE, 
                timeout=self.config.SERIAL_TIMEOUT
            )
            
            if self.config.VERBOSE_OUTPUT:
                print(f"✓ Connected to ESP32 on {port}")
            return True
        except Exception as e:
            print(f"✗ Serial connection failed: {e}")
            return False

    def setup_camera(self) -> bool:
        """Setup camera for object detection"""
        try:
            self.camera = cv2.VideoCapture(self.config.CAMERA_INDEX)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
                self.camera.set(cv2.CAP_PROP_FPS, self.config.CAMERA_FPS)
                
                if self.config.VERBOSE_OUTPUT:
                    print("✓ Camera initialized successfully")
                return True
            else:
                print("✗ Failed to open camera")
                return False
        except Exception as e:
            print(f"✗ Camera setup failed: {e}")
            return False

    def load_ai_model(self) -> bool:
        """Load YOLO AI model with enhanced security handling"""
        try:
            # PyTorch 2.6+ compatibility
            import torch
            original_load = torch.load
            def trusted_load(*args, **kwargs):
                kwargs['weights_only'] = False
                return original_load(*args, **kwargs)
            torch.load = trusted_load
            
            if not os.path.exists(self.config.MODEL_PATH):
                print(f"✗ Model file not found: {self.config.MODEL_PATH}")
                return False
            
            self.ai_model = YOLO(self.config.MODEL_PATH)
            
            if self.config.VERBOSE_OUTPUT:
                print("✓ AI model loaded successfully")
            return True
        except Exception as e:
            print(f"✗ AI model loading failed: {e}")
            return False

    def send_command(self, command: str) -> bool:
        """Send command to ESP32 with error handling"""
        if not self.serial_connection or not self.serial_connection.is_open:
            if self.config.VERBOSE_OUTPUT:
                print(f"Mock command: {command}")
            return True  # Allow testing without ESP32
        
        try:
            self.serial_connection.write(command.encode())
            if self.config.VERBOSE_OUTPUT:
                print(f"→ Command sent: {command}")
            return True
        except Exception as e:
            print(f"✗ Command send failed: {e}")
            return False

    def parse_sensor_data(self, data: str):
        """Parse sensor data from ESP32"""
        try:
            if "SENSORS:" in data:
                # Format: "SENSORS:F25.5,L30.2,R28.1,B35.0,T24.5,H60.0,S150"
                sensor_values = data.split(":")[1].split(",")
                self.sensor_data.front_distance = float(sensor_values[0][1:])
                self.sensor_data.left_distance = float(sensor_values[1][1:])
                self.sensor_data.right_distance = float(sensor_values[2][1:])
                self.sensor_data.rear_distance = float(sensor_values[3][1:])
                self.sensor_data.temperature = float(sensor_values[4][1:])
                self.sensor_data.humidity = float(sensor_values[5][1:])
                self.sensor_data.smoke_level = int(sensor_values[6][1:])
        except Exception as e:
            if self.config.VERBOSE_OUTPUT:
                print(f"Sensor data parsing error: {e}")

    def detect_objects_enhanced(self, frame) -> Tuple[bool, np.ndarray]:
        """Enhanced object detection with improved localization"""
        if not self.ai_model:
            return False, frame
        
        try:
            results = self.ai_model(
                frame, 
                imgsz=self.config.MODEL_INPUT_SIZE, 
                conf=self.config.MODEL_CONFIDENCE_THRESHOLD,
                verbose=False
            )[0]
            
            detections = sv.Detections.from_ultralytics(results)
            
            # Update detection data
            if len(detections) > 0:
                # Get the largest/closest detection (assuming closer objects are larger)
                largest_detection_idx = np.argmax(detections.area)
                bbox = detections.xyxy[largest_detection_idx]
                
                # Calculate object center and properties
                center_x = int((bbox[0] + bbox[2]) / 2)
                center_y = int((bbox[1] + bbox[3]) / 2)
                confidence = detections.confidence[largest_detection_idx]
                class_id = detections.class_id[largest_detection_idx] if len(detections.class_id) > 0 else 0
                area = detections.area[largest_detection_idx]
                
                # Update detection data
                self.detection_data.object_count = len(detections)
                self.detection_data.object_center_x = center_x
                self.detection_data.object_center_y = center_y
                self.detection_data.object_confidence = confidence
                self.detection_data.object_class_id = class_id
                self.detection_data.bbox_area = area
                self.detection_data.is_metal = class_id in self.config.METAL_CLASS_IDS
                
                # Track object history for stability
                self.target_object_history.append((center_x, center_y, time.time()))
                if len(self.target_object_history) > 10:
                    self.target_object_history.pop(0)
                
                # Reset lost count
                self.object_lost_count = 0
                
            else:
                self.detection_data.object_count = 0
                self.object_lost_count += 1
            
            # Annotate frame
            annotated_frame = self.box_annotator.annotate(scene=frame, detections=detections)
            annotated_frame = self.label_annotator.annotate(scene=annotated_frame, detections=detections)
            
            # Add custom annotations
            annotated_frame = self.add_custom_annotations(annotated_frame)
            
            return len(detections) > 0, annotated_frame
            
        except Exception as e:
            if self.config.VERBOSE_OUTPUT:
                print(f"Object detection error: {e}")
            return False, frame

    def add_custom_annotations(self, frame: np.ndarray) -> np.ndarray:
        """Add custom annotations to frame"""
        # Draw center crosshair
        cv2.line(frame, (self.frame_center_x-20, self.frame_center_y), 
                (self.frame_center_x+20, self.frame_center_y), (0, 255, 255), 2)
        cv2.line(frame, (self.frame_center_x, self.frame_center_y-20), 
                (self.frame_center_x, self.frame_center_y+20), (0, 255, 255), 2)
        
        # Draw detection zone
        zone_size = self.config.DETECTION_THRESHOLD
        cv2.rectangle(frame, 
                     (self.frame_center_x - zone_size, self.frame_center_y - zone_size),
                     (self.frame_center_x + zone_size, self.frame_center_y + zone_size),
                     (255, 255, 0), 2)
        
        # State and metrics overlay
        y_offset = 30
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (0, 255, 0)
        thickness = 2
        
        cv2.putText(frame, f"State: {self.current_state.value}", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        
        cv2.putText(frame, f"Objects: {self.detection_data.object_count}", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        
        cv2.putText(frame, f"Collected: {self.objects_collected} (M:{self.metal_objects}, NM:{self.non_metal_objects})", 
                   (10, y_offset), font, font_scale, color, thickness)
        y_offset += 30
        
        cv2.putText(frame, f"Sensors: F:{self.sensor_data.front_distance:.1f} L:{self.sensor_data.left_distance:.1f} R:{self.sensor_data.right_distance:.1f}", 
                   (10, y_offset), font, font_scale, color, thickness)
        
        # Show search pattern if searching
        if self.current_state == RobotState.SEARCHING:
            cv2.putText(frame, f"Search: Position {self.search_position_count}, Rotation {self.search_rotation_count}", 
                       (10, frame.shape[0] - 20), font, 0.6, (255, 0, 0), 2)
        
        return frame

    def is_obstacle_detected(self) -> bool:
        """Check if obstacles are detected by ultrasonic sensors"""
        return (self.sensor_data.front_distance < self.config.OBSTACLE_DISTANCE or
                self.sensor_data.left_distance < self.config.OBSTACLE_DISTANCE or
                self.sensor_data.right_distance < self.config.OBSTACLE_DISTANCE or
                self.sensor_data.rear_distance < self.config.OBSTACLE_DISTANCE)

    def is_emergency_stop_needed(self) -> bool:
        """Check if emergency stop is needed"""
        return self.sensor_data.front_distance < self.config.EMERGENCY_STOP_DISTANCE

    def get_safe_direction(self) -> MovementDirection:
        """Determine safe movement direction based on ultrasonic sensors"""
        distances = {
            MovementDirection.FORWARD: self.sensor_data.front_distance,
            MovementDirection.LEFT: self.sensor_data.left_distance,
            MovementDirection.RIGHT: self.sensor_data.right_distance,
            MovementDirection.BACKWARD: self.sensor_data.rear_distance
        }
        
        # Filter safe directions
        safe_directions = {k: v for k, v in distances.items() 
                          if v > self.config.OBSTACLE_DISTANCE}
        
        if safe_directions:
            # Choose direction with maximum clearance
            return max(safe_directions, key=safe_directions.get)
        else:
            # No safe direction, rotate to find space
            return MovementDirection.ROTATE_RIGHT

    def is_object_centered(self) -> bool:
        """Check if detected object is centered in camera view"""
        if self.detection_data.object_count == 0:
            return False
        
        x_diff = abs(self.detection_data.object_center_x - self.frame_center_x)
        return x_diff < self.config.DETECTION_THRESHOLD

    def get_alignment_direction(self) -> MovementDirection:
        """Get direction to align robot with detected object"""
        center_x = self.detection_data.object_center_x
        threshold = self.config.DETECTION_THRESHOLD
        
        if center_x < self.frame_center_x - threshold:
            return MovementDirection.ROTATE_LEFT
        elif center_x > self.frame_center_x + threshold:
            return MovementDirection.ROTATE_RIGHT
        else:
            return MovementDirection.FORWARD

    def enhanced_state_machine(self):
        """Enhanced state machine with improved logic"""
        with self.state_machine_lock:
            state_duration = time.time() - self.state_start_time
            
            # Emergency stop check
            if self.is_emergency_stop_needed() and self.current_state != RobotState.ERROR:
                self.send_command(MovementDirection.STOP.value)
                self.transition_to_state(RobotState.AVOIDING_OBSTACLE)
                return
            
            if self.current_state == RobotState.INITIALIZING:
                self.send_command(ArmPosition.HOME.value)
                self.send_command(BucketPosition.NEUTRAL.value)
                self.transition_to_state(RobotState.SEARCHING)
                
            elif self.current_state == RobotState.SEARCHING:
                self.execute_search_behavior(state_duration)
                
            elif self.current_state == RobotState.OBJECT_DETECTED:
                self.execute_object_tracking(state_duration)
                
            elif self.current_state == RobotState.APPROACHING:
                self.execute_approach_behavior(state_duration)
                
            elif self.current_state == RobotState.POSITIONING:
                self.execute_positioning_behavior(state_duration)
                
            elif self.current_state == RobotState.PICKING:
                self.execute_picking_sequence(state_duration)
                
            elif self.current_state == RobotState.CLASSIFYING:
                self.execute_classification(state_duration)
                
            elif self.current_state == RobotState.DEPOSITING:
                self.execute_depositing_sequence(state_duration)
                
            elif self.current_state == RobotState.AVOIDING_OBSTACLE:
                self.execute_obstacle_avoidance(state_duration)

    def execute_search_behavior(self, state_duration: float):
        """Execute systematic search behavior"""
        rotation_time = self.config.ROTATION_TIME / self.config.SEARCH_ROTATIONS
        
        if self.search_rotation_count < self.config.SEARCH_ROTATIONS:
            # Rotate to search for objects
            self.send_command(MovementDirection.ROTATE_RIGHT.value)
            
            if state_duration > rotation_time:
                self.search_rotation_count += 1
                self.state_start_time = time.time()
                
                if self.config.VERBOSE_OUTPUT:
                    print(f"Completed rotation {self.search_rotation_count}/{self.config.SEARCH_ROTATIONS}")
        else:
            # Completed rotations at current position, move to next search point
            self.search_rotation_count = 0
            
            if self.search_position_count < len(self.search_pattern):
                # Move to next position in search pattern
                safe_direction = self.get_safe_direction()
                self.send_command(safe_direction.value)
                
                if state_duration > self.config.MOVEMENT_DURATION:
                    self.search_position_count += 1
                    self.transition_to_state(RobotState.SEARCHING)
                    
                    if self.config.VERBOSE_OUTPUT:
                        print(f"Moving to search position {self.search_position_count}")
            else:
                # Completed all search positions, return to start
                self.search_position_count = 0
                self.transition_to_state(RobotState.RETURNING_HOME)

    def execute_object_tracking(self, state_duration: float):
        """Execute object tracking and alignment"""
        if self.detection_data.object_count == 0:
            self.object_lost_count += 1
            if self.object_lost_count > self.max_object_lost:
                # Object lost, return to searching
                if self.config.VERBOSE_OUTPUT:
                    print("Object lost, returning to search")
                self.transition_to_state(RobotState.SEARCHING)
            return
        
        if not self.is_object_centered():
            # Align with object
            alignment_direction = self.get_alignment_direction()
            self.send_command(alignment_direction.value)
            
            if self.config.VERBOSE_OUTPUT:
                print(f"Aligning with object: {alignment_direction.value}")
        else:
            # Object centered, start approaching
            if self.config.VERBOSE_OUTPUT:
                print("Object centered, starting approach")
            self.transition_to_state(RobotState.APPROACHING)

    def execute_approach_behavior(self, state_duration: float):
        """Execute approach to object"""
        if self.detection_data.object_count == 0:
            self.transition_to_state(RobotState.SEARCHING)
            return
        
        if self.sensor_data.front_distance <= self.config.OBJECT_APPROACH_DISTANCE:
            # Close enough to object
            self.send_command(MovementDirection.STOP.value)
            self.transition_to_state(RobotState.POSITIONING)
            
            if self.config.VERBOSE_OUTPUT:
                print(f"Reached object (distance: {self.sensor_data.front_distance:.1f}cm)")
        elif self.is_obstacle_detected():
            # Obstacle detected during approach
            self.transition_to_state(RobotState.AVOIDING_OBSTACLE)
        else:
            # Continue approaching
            if self.is_object_centered():
                self.send_command(MovementDirection.FORWARD.value)
            else:
                alignment_direction = self.get_alignment_direction()
                self.send_command(alignment_direction.value)

    def execute_positioning_behavior(self, state_duration: float):
        """Execute fine positioning before picking"""
        if state_duration > 1.0:  # Allow time for robot to stabilize
            if self.config.VERBOSE_OUTPUT:
                print("Positioning complete, starting pick sequence")
            self.transition_to_state(RobotState.PICKING)

    def execute_picking_sequence(self, state_duration: float):
        """Execute picking sequence with timing"""
        if state_duration < 1.0:
            self.send_command(ArmPosition.APPROACH.value)
        elif state_duration < 2.0:
            self.send_command(ArmPosition.GRAB_OPEN.value)
        elif state_duration < 3.0:
            self.send_command(ArmPosition.GRAB_CLOSE.value)
        elif state_duration < 4.0:
            self.send_command(ArmPosition.LIFT_UP.value)
        else:
            if self.config.VERBOSE_OUTPUT:
                print("Pick sequence complete, moving to classification")
            self.transition_to_state(RobotState.CLASSIFYING)

    def execute_classification(self, state_duration: float):
        """Execute object classification"""
        self.send_command(ArmPosition.CLASSIFY.value)
        
        if state_duration > 1.0:
            # Classification based on AI detection data
            if self.detection_data.object_confidence > self.config.CONFIDENCE_THRESHOLD:
                object_type = "Metal" if self.detection_data.is_metal else "Non-Metal"
                if self.config.VERBOSE_OUTPUT:
                    print(f"Object classified as: {object_type} (confidence: {self.detection_data.object_confidence:.2f})")
            
            self.transition_to_state(RobotState.DEPOSITING)

    def execute_depositing_sequence(self, state_duration: float):
        """Execute object depositing sequence"""
        if state_duration < 1.0:
            # Position bucket
            if self.detection_data.is_metal:
                self.send_command(BucketPosition.METAL.value)
            else:
                self.send_command(BucketPosition.NON_METAL.value)
        elif state_duration < 2.0:
            # Wait for bucket positioning
            pass
        elif state_duration < 3.0:
            # Release object
            self.send_command(ArmPosition.GRAB_OPEN.value)
            
            # Update statistics
            self.objects_collected += 1
            if self.detection_data.is_metal:
                self.metal_objects += 1
            else:
                self.non_metal_objects += 1
                
            if self.config.VERBOSE_OUTPUT:
                print(f"Object deposited! Total collected: {self.objects_collected}")
        else:
            # Return to home position and resume searching
            self.send_command(ArmPosition.HOME.value)
            self.send_command(BucketPosition.NEUTRAL.value)
            self.transition_to_state(RobotState.SEARCHING)

    def execute_obstacle_avoidance(self, state_duration: float):
        """Execute obstacle avoidance behavior"""
        if not self.is_obstacle_detected():
            # Obstacle cleared
            if self.config.VERBOSE_OUTPUT:
                print("Obstacle cleared, resuming previous state")
            self.transition_to_state(self.previous_state)
        else:
            # Continue avoiding obstacle
            safe_direction = self.get_safe_direction()
            self.send_command(safe_direction.value)
            
            if self.config.VERBOSE_OUTPUT and state_duration % 2.0 < 0.1:  # Print every 2 seconds
                print(f"Avoiding obstacle, moving {safe_direction.value}")

    def transition_to_state(self, new_state: RobotState):
        """Transition to new robot state with logging"""
        if self.config.LOG_STATE_TRANSITIONS:
            print(f"State transition: {self.current_state.value} → {new_state.value}")
        
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()

    # Threading methods remain the same as in the previous version
    def serial_reader_thread(self):
        """Background thread for reading ESP32 data"""
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.readline().decode('utf-8').strip()
                    if data:
                        if self.config.VERBOSE_OUTPUT:
                            print(f"ESP32: {data}")
                        self.parse_sensor_data(data)
            except Exception as e:
                if self.config.VERBOSE_OUTPUT:
                    print(f"Serial read error: {e}")
            time.sleep(1.0 / 2)  # 2 Hz sensor reading

    def vision_processing_thread(self):
        """Background thread for camera and AI processing"""
        while self.running:
            try:
                if self.camera:
                    ret, frame = self.camera.read()
                    if ret:
                        has_objects, annotated_frame = self.detect_objects_enhanced(frame)
                        
                        # State transitions based on detection
                        if has_objects and self.current_state == RobotState.SEARCHING:
                            self.transition_to_state(RobotState.OBJECT_DETECTED)
                        
                        # Display frame
                        if self.config.SHOW_CAMERA_FEED:
                            cv2.imshow('Enhanced Autonomous Garbage Collector', annotated_frame)
                            
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                self.stop()
            except Exception as e:
                if self.config.VERBOSE_OUTPUT:
                    print(f"Vision processing error: {e}")
            time.sleep(1.0 / self.config.VISION_LOOP_FREQUENCY)

    def main_control_thread(self):
        """Main control loop thread"""
        while self.running:
            try:
                self.enhanced_state_machine()
                time.sleep(1.0 / self.config.CONTROL_LOOP_FREQUENCY)
            except Exception as e:
                print(f"Control loop error: {e}")
                self.transition_to_state(RobotState.ERROR)

    def start(self):
        """Start the enhanced autonomous robot system"""
        print("🤖 Starting Enhanced Autonomous Garbage Collector...")
        print(f"📋 Configuration loaded:")
        print(f"   - Model: {self.config.MODEL_PATH}")
        print(f"   - Camera: {self.config.CAMERA_WIDTH}x{self.config.CAMERA_HEIGHT}")
        print(f"   - Search Pattern: {len(self.search_pattern)} positions")
        print(f"   - Metal Classes: {self.config.METAL_CLASS_IDS}")
        
        # Initialize components
        if not self.setup_serial_connection():
            print("⚠️ Serial connection failed - continuing in mock mode")
        
        if not self.setup_camera():
            print("❌ Camera setup failed - cannot continue")
            return False
        
        if not self.load_ai_model():
            print("❌ AI model loading failed - cannot continue")
            return False
        
        print("✅ All systems initialized successfully")
        
        # Start threads
        self.running = True
        
        threads = [
            threading.Thread(target=self.serial_reader_thread, daemon=True),
            threading.Thread(target=self.vision_processing_thread, daemon=True),
            threading.Thread(target=self.main_control_thread, daemon=True)
        ]
        
        for thread in threads:
            thread.start()
        
        print("🚀 Robot is now autonomous! Press 'q' in camera window or Ctrl+C to stop")
        
        try:
            while self.running:
                time.sleep(5)  # Status update every 5 seconds
                uptime = time.time() - self.session_start_time
                print(f"[{uptime:.0f}s] State: {self.current_state.value} | "
                      f"Objects: {self.objects_collected} | "
                      f"Metal: {self.metal_objects} | Non-Metal: {self.non_metal_objects}")
        except KeyboardInterrupt:
            print("\n🛑 Shutting down robot...")
        
        self.stop()
        return True

    def stop(self):
        """Stop the robot and cleanup"""
        self.running = False
        
        if self.serial_connection:
            self.send_command(MovementDirection.STOP.value)
            self.send_command(ArmPosition.HOME.value)
            self.serial_connection.close()
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()
        
        # Print final statistics
        session_time = time.time() - self.session_start_time
        print(f"\n📊 Session Statistics:")
        print(f"   - Runtime: {session_time:.1f} seconds")
        print(f"   - Objects Collected: {self.objects_collected}")
        print(f"   - Metal Objects: {self.metal_objects}")
        print(f"   - Non-Metal Objects: {self.non_metal_objects}")
        print("✅ Robot shutdown complete")

def main():
    """Main function"""
    print("🤖 Micro Lab Garbage Collector - Enhanced Autonomous System")
    print("=" * 60)
    
    robot = EnhancedAutonomousRobot()
    success = robot.start()
    
    if not success:
        print("❌ Robot startup failed")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())