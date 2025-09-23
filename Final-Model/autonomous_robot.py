#!/usr/bin/env python3
"""
Micro Lab Garbage Collector - Autonomous Robot System
Comprehensive garbage collection robot with object detection, navigation, and classification
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

# Suppress warnings
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
    HOME = "A0"           # Arm at home position
    APPROACH = "A1"       # Arm approaching object
    GRAB_OPEN = "A2"      # Gripper open, ready to grab
    GRAB_CLOSE = "A3"     # Gripper closed, holding object
    LIFT_UP = "A4"        # Lift object up
    CLASSIFY = "A5"       # Move to classification position

class BucketPosition(Enum):
    """Bucket positions for object classification"""
    METAL = "B1"          # Metal objects bucket
    NON_METAL = "B2"      # Non-metal objects bucket
    NEUTRAL = "B0"        # Neutral position

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
    is_metal: bool = False

class AutonomousRobot:
    """Main robot control system with threading"""
    
    def __init__(self):
        # System configuration
        self.model_path = "best.pt"
        self.camera_index = 0
        self.serial_port = "AUTO"
        self.baud_rate = 115200
        
        # System components
        self.serial_connection: Optional[serial.Serial] = None
        self.camera: Optional[cv2.VideoCapture] = None
        self.ai_model: Optional[YOLO] = None
        
        # Threading components
        self.running = False
        self.command_queue = queue.Queue()
        self.sensor_data = SensorData()
        self.detection_data = DetectionData()
        
        # Robot state
        self.current_state = RobotState.INITIALIZING
        self.previous_state = RobotState.INITIALIZING
        self.state_start_time = time.time()
        
        # Navigation variables
        self.search_rotation_count = 0
        self.search_position_count = 0
        self.last_movement_time = time.time()
        self.movement_duration = 2.0  # seconds
        
        # Object tracking
        self.target_object_x = 0
        self.target_object_y = 0
        self.object_approach_distance = 15.0  # cm
        
        # Safety parameters
        self.obstacle_distance = 20.0  # cm
        self.frame_center_x = 320  # Camera frame center
        self.frame_center_y = 240
        self.detection_threshold = 50  # pixels from center
        
        # Statistics
        self.objects_collected = 0
        self.metal_objects = 0
        self.non_metal_objects = 0
        
        # Initialize annotators
        self.box_annotator = sv.BoxAnnotator()
        self.label_annotator = sv.LabelAnnotator()

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
            if self.serial_port == "AUTO":
                auto_port = self.find_esp32_port()
                port = auto_port if auto_port else "COM3"
            else:
                port = self.serial_port
            
            self.serial_connection = serial.Serial(port, self.baud_rate, timeout=1)
            print(f"✓ Connected to ESP32 on {port}")
            return True
        except Exception as e:
            print(f"✗ Serial connection failed: {e}")
            return False

    def setup_camera(self) -> bool:
        """Setup camera for object detection"""
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                print("✓ Camera initialized successfully")
                return True
            else:
                print("✗ Failed to open camera")
                return False
        except Exception as e:
            print(f"✗ Camera setup failed: {e}")
            return False

    def load_ai_model(self) -> bool:
        """Load YOLO AI model"""
        try:
            # PyTorch 2.6+ compatibility
            import torch
            original_load = torch.load
            def trusted_load(*args, **kwargs):
                kwargs['weights_only'] = False
                return original_load(*args, **kwargs)
            torch.load = trusted_load
            
            if not os.path.exists(self.model_path):
                print(f"✗ Model file not found: {self.model_path}")
                return False
            
            self.ai_model = YOLO(self.model_path)
            print("✓ AI model loaded successfully")
            return True
        except Exception as e:
            print(f"✗ AI model loading failed: {e}")
            return False

    def send_command(self, command: str) -> bool:
        """Send command to ESP32"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return False
        
        try:
            self.serial_connection.write(command.encode())
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
            print(f"Sensor data parsing error: {e}")

    def detect_objects(self, frame) -> Tuple[bool, np.ndarray]:
        """Detect objects in camera frame"""
        if not self.ai_model:
            return False, frame
        
        try:
            results = self.ai_model(frame, imgsz=480, verbose=False)[0]
            detections = sv.Detections.from_ultralytics(results)
            
            # Update detection data
            if len(detections) > 0:
                # Get the largest/closest detection
                largest_detection_idx = np.argmax(detections.area)
                bbox = detections.xyxy[largest_detection_idx]
                
                # Calculate object center
                self.detection_data.object_center_x = int((bbox[0] + bbox[2]) / 2)
                self.detection_data.object_center_y = int((bbox[1] + bbox[3]) / 2)
                self.detection_data.object_count = len(detections)
                self.detection_data.object_confidence = detections.confidence[largest_detection_idx]
                
                # Simple metal classification (placeholder - enhance based on your model)
                class_id = detections.class_id[largest_detection_idx] if len(detections.class_id) > 0 else 0
                self.detection_data.is_metal = class_id in [0, 1, 2]  # Adjust based on your classes
                
                self.target_object_x = self.detection_data.object_center_x
                self.target_object_y = self.detection_data.object_center_y
            else:
                self.detection_data.object_count = 0
            
            # Annotate frame
            annotated_frame = self.box_annotator.annotate(scene=frame, detections=detections)
            annotated_frame = self.label_annotator.annotate(scene=annotated_frame, detections=detections)
            
            return len(detections) > 0, annotated_frame
            
        except Exception as e:
            print(f"Object detection error: {e}")
            return False, frame

    def is_obstacle_detected(self) -> bool:
        """Check if obstacles are detected by ultrasonic sensors"""
        return (self.sensor_data.front_distance < self.obstacle_distance or
                self.sensor_data.left_distance < self.obstacle_distance or
                self.sensor_data.right_distance < self.obstacle_distance or
                self.sensor_data.rear_distance < self.obstacle_distance)

    def get_safe_direction(self) -> MovementDirection:
        """Determine safe movement direction based on ultrasonic sensors"""
        distances = {
            MovementDirection.FORWARD: self.sensor_data.front_distance,
            MovementDirection.LEFT: self.sensor_data.left_distance,
            MovementDirection.RIGHT: self.sensor_data.right_distance,
            MovementDirection.BACKWARD: self.sensor_data.rear_distance
        }
        
        # Filter safe directions
        safe_directions = {k: v for k, v in distances.items() if v > self.obstacle_distance}
        
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
        
        x_diff = abs(self.target_object_x - self.frame_center_x)
        return x_diff < self.detection_threshold

    def get_alignment_direction(self) -> MovementDirection:
        """Get direction to align robot with detected object"""
        if self.target_object_x < self.frame_center_x - self.detection_threshold:
            return MovementDirection.ROTATE_LEFT
        elif self.target_object_x > self.frame_center_x + self.detection_threshold:
            return MovementDirection.ROTATE_RIGHT
        else:
            return MovementDirection.FORWARD

    def state_machine(self):
        """Main state machine for robot behavior"""
        state_duration = time.time() - self.state_start_time
        
        if self.current_state == RobotState.INITIALIZING:
            self.send_command(ArmPosition.HOME.value)
            self.send_command(BucketPosition.NEUTRAL.value)
            self.transition_to_state(RobotState.SEARCHING)
            
        elif self.current_state == RobotState.SEARCHING:
            # Rotate 360 degrees twice to search for objects
            if self.search_rotation_count < 2:
                self.send_command(MovementDirection.ROTATE_RIGHT.value)
                if state_duration > 8.0:  # 360 degree rotation time
                    self.search_rotation_count += 1
                    self.state_start_time = time.time()
            else:
                # No objects found, move to new position
                self.search_rotation_count = 0
                safe_direction = self.get_safe_direction()
                self.send_command(safe_direction.value)
                
                if state_duration > self.movement_duration:
                    self.search_position_count += 1
                    self.transition_to_state(RobotState.SEARCHING)
                    
        elif self.current_state == RobotState.OBJECT_DETECTED:
            # Object detected, start approaching
            if not self.is_object_centered():
                # Align with object
                alignment_direction = self.get_alignment_direction()
                self.send_command(alignment_direction.value)
            else:
                # Object centered, transition to approaching
                self.transition_to_state(RobotState.APPROACHING)
                
        elif self.current_state == RobotState.APPROACHING:
            if self.detection_data.object_count == 0:
                # Lost object, return to searching
                self.transition_to_state(RobotState.SEARCHING)
            elif self.sensor_data.front_distance <= self.object_approach_distance:
                # Close enough to object
                self.send_command(MovementDirection.STOP.value)
                self.transition_to_state(RobotState.POSITIONING)
            elif self.is_obstacle_detected():
                # Obstacle detected, avoid
                self.transition_to_state(RobotState.AVOIDING_OBSTACLE)
            else:
                # Continue approaching
                if self.is_object_centered():
                    self.send_command(MovementDirection.FORWARD.value)
                else:
                    alignment_direction = self.get_alignment_direction()
                    self.send_command(alignment_direction.value)
                    
        elif self.current_state == RobotState.POSITIONING:
            # Fine positioning before picking
            self.send_command(MovementDirection.STOP.value)
            self.transition_to_state(RobotState.PICKING)
            
        elif self.current_state == RobotState.PICKING:
            # Execute picking sequence
            if state_duration < 1.0:
                self.send_command(ArmPosition.APPROACH.value)
            elif state_duration < 2.0:
                self.send_command(ArmPosition.GRAB_OPEN.value)
            elif state_duration < 3.0:
                self.send_command(ArmPosition.GRAB_CLOSE.value)
            elif state_duration < 4.0:
                self.send_command(ArmPosition.LIFT_UP.value)
            else:
                self.transition_to_state(RobotState.CLASSIFYING)
                
        elif self.current_state == RobotState.CLASSIFYING:
            # Move to classification position and analyze
            self.send_command(ArmPosition.CLASSIFY.value)
            
            # Simple classification based on detection data
            if state_duration > 1.0:
                self.transition_to_state(RobotState.DEPOSITING)
                
        elif self.current_state == RobotState.DEPOSITING:
            # Deposit object in appropriate bucket
            if self.detection_data.is_metal:
                self.send_command(BucketPosition.METAL.value)
                self.metal_objects += 1
            else:
                self.send_command(BucketPosition.NON_METAL.value)
                self.non_metal_objects += 1
            
            # Wait for bucket positioning then release
            if state_duration > 2.0:
                self.send_command(ArmPosition.GRAB_OPEN.value)
                self.objects_collected += 1
                
                # Return to home position and search
                if state_duration > 3.0:
                    self.send_command(ArmPosition.HOME.value)
                    self.send_command(BucketPosition.NEUTRAL.value)
                    self.transition_to_state(RobotState.SEARCHING)
                    
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            # Obstacle avoidance behavior
            safe_direction = self.get_safe_direction()
            self.send_command(safe_direction.value)
            
            if state_duration > 1.0 and not self.is_obstacle_detected():
                # Obstacle cleared, return to previous state
                self.transition_to_state(self.previous_state)

    def transition_to_state(self, new_state: RobotState):
        """Transition to new robot state"""
        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()
        print(f"State transition: {self.previous_state.value} → {new_state.value}")

    def serial_reader_thread(self):
        """Background thread for reading ESP32 data"""
        while self.running:
            try:
                if self.serial_connection and self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.readline().decode('utf-8').strip()
                    if data:
                        print(f"ESP32: {data}")
                        self.parse_sensor_data(data)
            except Exception as e:
                print(f"Serial read error: {e}")
            time.sleep(0.1)

    def vision_processing_thread(self):
        """Background thread for camera and AI processing"""
        while self.running:
            try:
                if self.camera:
                    ret, frame = self.camera.read()
                    if ret:
                        has_objects, annotated_frame = self.detect_objects(frame)
                        
                        # State transitions based on detection
                        if has_objects and self.current_state == RobotState.SEARCHING:
                            self.transition_to_state(RobotState.OBJECT_DETECTED)
                        
                        # Display frame with annotations
                        cv2.putText(annotated_frame, f"State: {self.current_state.value}", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated_frame, f"Objects: {self.detection_data.object_count}", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated_frame, f"Collected: {self.objects_collected}", 
                                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(annotated_frame, f"Metal: {self.metal_objects} | Non-Metal: {self.non_metal_objects}", 
                                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # Draw center crosshair
                        cv2.line(annotated_frame, (self.frame_center_x-20, self.frame_center_y), 
                                (self.frame_center_x+20, self.frame_center_y), (0, 255, 255), 2)
                        cv2.line(annotated_frame, (self.frame_center_x, self.frame_center_y-20), 
                                (self.frame_center_x, self.frame_center_y+20), (0, 255, 255), 2)
                        
                        cv2.imshow('Autonomous Garbage Collector', annotated_frame)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.stop()
            except Exception as e:
                print(f"Vision processing error: {e}")
            time.sleep(0.033)  # ~30 FPS

    def main_control_thread(self):
        """Main control loop thread"""
        while self.running:
            try:
                self.state_machine()
                time.sleep(0.1)  # 10Hz control loop
            except Exception as e:
                print(f"Control loop error: {e}")
                self.transition_to_state(RobotState.ERROR)

    def start(self):
        """Start the autonomous robot system"""
        print("🤖 Starting Autonomous Garbage Collector...")
        
        # Initialize components
        if not self.setup_serial_connection():
            print("⚠️ Serial connection failed - continuing without ESP32")
        
        if not self.setup_camera():
            print("❌ Camera setup failed - cannot continue")
            return False
        
        if not self.load_ai_model():
            print("❌ AI model loading failed - cannot continue")
            return False
        
        print("✅ All systems initialized successfully")
        
        # Start threads
        self.running = True
        
        # Serial communication thread
        serial_thread = threading.Thread(target=self.serial_reader_thread, daemon=True)
        serial_thread.start()
        
        # Vision processing thread
        vision_thread = threading.Thread(target=self.vision_processing_thread, daemon=True)
        vision_thread.start()
        
        # Main control thread
        control_thread = threading.Thread(target=self.main_control_thread, daemon=True)
        control_thread.start()
        
        print("🚀 Robot is now autonomous! Press 'q' in camera window or Ctrl+C to stop")
        
        try:
            # Keep main thread alive
            while self.running:
                time.sleep(1)
                print(f"Status: {self.current_state.value} | Objects: {self.objects_collected} | "
                      f"Metal: {self.metal_objects} | Non-Metal: {self.non_metal_objects}")
        except KeyboardInterrupt:
            print("\n🛑 Shutting down robot...")
        
        self.stop()
        return True

    def stop(self):
        """Stop the robot and cleanup"""
        self.running = False
        
        # Send stop command
        if self.serial_connection:
            self.send_command(MovementDirection.STOP.value)
            self.send_command(ArmPosition.HOME.value)
            self.serial_connection.close()
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()
        print("✅ Robot shutdown complete")

def main():
    """Main function"""
    robot = AutonomousRobot()
    robot.start()

if __name__ == "__main__":
    main()