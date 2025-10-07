#!/usr/bin/env python3

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
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Tuple, List
import json
from dotenv import load_dotenv

# Load configuration
load_dotenv('robot_config.env')
warnings.filterwarnings("ignore", category=FutureWarning)

class RobotState(Enum):
    """Robot operational states matching ESP32 states"""
    STOPPED = "stopped"
    SEARCHING = "searching"
    TRACKING = "tracking"
    ULTRASONIC_RANGE = "ultrasonic_range"
    COLLECTING = "collecting"

class SystemStatus(Enum):
    """System control status"""
    EMERGENCY_STOP = "emergency_stop"
    PAUSED = "paused"
    RUNNING = "running"

@dataclass
class CameraCalibration:
    """Camera calibration data for distance measurement"""
    focal_length_x: float
    focal_length_y: float
    known_width: float  # Known object width in meters
    known_height: float  # Known object height in meters
    is_calibrated: bool

@dataclass
class ObjectDetection:
    """Object detection result"""
    center_x: int
    center_y: int
    width: int
    height: int
    confidence: float
    class_name: str
    distance: float

class AdvancedAutonomousRobot:
    def __init__(self):
        """Initialize the Advanced Autonomous Robot System"""
        self.robot_state = RobotState.STOPPED
        self.system_status = SystemStatus.PAUSED
        
        # Camera and detection settings
        self.camera_index = int(os.getenv('CAMERA_INDEX', '0'))
        self.frame_width = int(os.getenv('FRAME_WIDTH', '640'))
        self.frame_height = int(os.getenv('FRAME_HEIGHT', '480'))
        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2
        self.center_tolerance = int(os.getenv('CENTER_TOLERANCE', '50'))
        
        # Detection settings
        self.confidence_threshold = float(os.getenv('CONFIDENCE_THRESHOLD', '0.5'))
        self.target_classes = os.getenv('TARGET_CLASSES', 'bottle,cup,can').split(',')
        
        # Communication settings
        self.serial_port = None
        self.serial_baudrate = int(os.getenv('SERIAL_BAUDRATE', '115200'))
        self.serial_timeout = float(os.getenv('SERIAL_TIMEOUT', '1.0'))
        
        # Threading and control
        self.camera_thread = None
        self.communication_thread = None
        self.detection_thread = None
        self.frame_queue = queue.Queue(maxsize=10)
        self.detection_queue = queue.Queue(maxsize=5)
        self.running = False
        
        # Camera and AI components
        self.camera = None
        self.model = None
        self.annotator = sv.BoxAnnotator()
        
        # Distance measurement
        self.calibration = None
        self.last_detection = None
        self.distance_measurement_ready = False
        
        # Performance monitoring
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.detection_count = 0
        
        print("🤖 Advanced Autonomous Robot System Initialized")
        print("="*60)
        
    def initialize_system(self):
        """Initialize all system components"""
        try:
            print("🚀 Initializing system components...")
            
            # Load AI model
            self._load_ai_model()
            
            # Initialize camera
            self._initialize_camera()
            
            # Load calibration data
            self._load_calibration()
            
            # Initialize serial communication
            self._initialize_serial()
            
            self.robot_state = RobotState.STOPPED
            print("✅ System initialization completed successfully!")
            return True
            
        except Exception as e:
            print(f"❌ System initialization failed: {e}")
            self.robot_state = RobotState.STOPPED
            return False
    
    def _load_ai_model(self):
        """Load YOLO model for object detection"""
        try:
            model_path = os.getenv('MODEL_PATH', 'best.pt')
            if not os.path.exists(model_path):
                model_path = 'yolov8n.pt'  # Fallback to default model
                
            print(f"📦 Loading AI model: {model_path}")
            self.model = YOLO(model_path)
            print(f"✅ AI model loaded successfully")
            
        except Exception as e:
            print(f"❌ Failed to load AI model: {e}")
            raise
    
    def _initialize_camera(self):
        """Initialize camera with optimal settings"""
        try:
            print(f"📹 Initializing camera (index: {self.camera_index})")
            self.camera = cv2.VideoCapture(self.camera_index)
            
            if not self.camera.isOpened():
                raise Exception(f"Failed to open camera {self.camera_index}")
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"✅ Camera initialized: {actual_width}x{actual_height}")
            
        except Exception as e:
            print(f"❌ Camera initialization failed: {e}")
            raise
    
    def _load_calibration(self):
        """Load camera calibration data for distance measurement"""
        try:
            calibration_file = 'camera_calibration.json'
            if os.path.exists(calibration_file):
                with open(calibration_file, 'r') as f:
                    cal_data = json.load(f)
                
                self.calibration = CameraCalibration(
                    focal_length_x=cal_data.get('focal_length_x', 500.0),
                    focal_length_y=cal_data.get('focal_length_y', 500.0),
                    known_width=cal_data.get('known_width', 0.1),
                    known_height=cal_data.get('known_height', 0.15),
                    is_calibrated=cal_data.get('is_calibrated', False)
                )
                
                if self.calibration.is_calibrated:
                    self.distance_measurement_ready = True
                    print("✅ Camera calibration loaded - Distance measurement: READY")
                else:
                    print("⚠️  Camera calibration found but not validated")
            else:
                print("⚠️  No camera calibration found - Distance measurement disabled")
                self.calibration = CameraCalibration(500.0, 500.0, 0.1, 0.15, False)
                
        except Exception as e:
            print(f"❌ Failed to load calibration: {e}")
            self.calibration = CameraCalibration(500.0, 500.0, 0.1, 0.15, False)
    
    def _initialize_serial(self):
        """Initialize serial communication with ESP32"""
        try:
            # Auto-detect Arduino port
            arduino_port = self._find_arduino_port()
            if not arduino_port:
                print("⚠️  No Arduino/ESP32 found - Running in simulation mode")
                return
            
            print(f"🔌 Connecting to ESP32: {arduino_port}")
            self.serial_port = serial.Serial(
                port=arduino_port,
                baudrate=self.serial_baudrate,
                timeout=self.serial_timeout,
                write_timeout=1.0
            )
            
            # Wait for ESP32 to initialize
            time.sleep(2)
            
            # Clear any pending data
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            
            print("✅ Serial communication established")
            
        except Exception as e:
            print(f"❌ Serial initialization failed: {e}")
            self.serial_port = None
    
    def _find_arduino_port(self):
        """Auto-detect Arduino/ESP32 port"""
        arduino_keywords = ['Arduino', 'ESP32', 'CH340', 'CP210', 'USB-SERIAL']
        
        for port in serial.tools.list_ports.comports():
            port_description = f"{port.description} {port.manufacturer or ''}"
            if any(keyword.lower() in port_description.lower() for keyword in arduino_keywords):
                return port.device
        return None
    
    def calculate_distance(self, detection_width, detection_height):
        """Calculate distance to object using camera calibration"""
        if not self.distance_measurement_ready:
            return 999.0  # Invalid distance
        
        try:
            # Use width-based calculation (generally more reliable)
            distance = (self.calibration.known_width * self.calibration.focal_length_x) / detection_width
            
            # Clamp to reasonable range
            distance = max(0.05, min(distance, 10.0))
            return distance
            
        except (ZeroDivisionError, ValueError):
            return 999.0
    
    def send_command(self, command):
        """Send command to ESP32"""
        if self.serial_port and self.serial_port.is_open:
            try:
                message = f"{command}\n"
                self.serial_port.write(message.encode())
                self.serial_port.flush()
            except Exception as e:
                print(f"❌ Serial communication error: {e}")
    
    def process_detections(self, frame):
        """Process object detections and calculate distances"""
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        detections = []
        
        for result in results:
            if result.boxes is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                class_ids = result.boxes.cls.cpu().numpy().astype(int)
                
                for box, conf, class_id in zip(boxes, confidences, class_ids):
                    class_name = self.model.names[class_id]
                    
                    # Filter by target classes
                    if class_name in self.target_classes:
                        x1, y1, x2, y2 = box
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        width = int(x2 - x1)
                        height = int(y2 - y1)
                        
                        # Calculate distance
                        distance = self.calculate_distance(width, height)
                        
                        detection = ObjectDetection(
                            center_x=center_x,
                            center_y=center_y,
                            width=width,
                            height=height,
                            confidence=conf,
                            class_name=class_name,
                            distance=distance
                        )
                        
                        detections.append(detection)
        
        return detections
    
    def draw_annotations(self, frame, detections):
        """Draw detection annotations on frame"""
        annotated_frame = frame.copy()
        
        # Draw center crosshair
        cv2.line(annotated_frame, (self.frame_center_x - 20, self.frame_center_y), 
                (self.frame_center_x + 20, self.frame_center_y), (0, 255, 0), 2)
        cv2.line(annotated_frame, (self.frame_center_x, self.frame_center_y - 20), 
                (self.frame_center_x, self.frame_center_y + 20), (0, 255, 0), 2)
        
        # Draw center tolerance zone
        cv2.rectangle(annotated_frame, 
                     (self.frame_center_x - self.center_tolerance, self.frame_center_y - 50),
                     (self.frame_center_x + self.center_tolerance, self.frame_center_y + 50),
                     (0, 255, 0), 1)
        
        for detection in detections:
            # Draw bounding box
            x1 = detection.center_x - detection.width // 2
            y1 = detection.center_y - detection.height // 2
            x2 = detection.center_x + detection.width // 2
            y2 = detection.center_y + detection.height // 2
            
            # Color based on centering
            is_centered = abs(detection.center_x - self.frame_center_x) <= self.center_tolerance
            color = (0, 255, 0) if is_centered else (0, 0, 255)
            
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            cv2.circle(annotated_frame, (detection.center_x, detection.center_y), 5, color, -1)
            
            # Draw labels
            label = f"{detection.class_name} {detection.confidence:.2f}"
            if detection.distance < 999.0:
                label += f" | {detection.distance:.2f}m"
            
            cv2.putText(annotated_frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return annotated_frame
    
    def draw_status_info(self, frame):
        """Draw system status information"""
        # Status background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Status text
        status_texts = [
            f"State: {self.robot_state.value.upper()}",
            f"System: {self.system_status.value.upper()}",
            f"Distance: {'READY' if self.distance_measurement_ready else 'NOT CALIBRATED'}",
            f"Serial: {'CONNECTED' if self.serial_port else 'DISCONNECTED'}",
            f"Detections: {self.detection_count}"
        ]
        
        for i, text in enumerate(status_texts):
            cv2.putText(frame, text, (15, 30 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def camera_capture_thread(self):
        """Camera capture thread"""
        while self.running:
            try:
                ret, frame = self.camera.read()
                if ret:
                    if not self.frame_queue.full():
                        self.frame_queue.put(frame)
                    
                    # FPS calculation
                    self.fps_counter += 1
                    if time.time() - self.fps_start_time >= 1.0:
                        fps = self.fps_counter / (time.time() - self.fps_start_time)
                        self.fps_counter = 0
                        self.fps_start_time = time.time()
                        
            except Exception as e:
                print(f"❌ Camera capture error: {e}")
                time.sleep(0.1)
    
    def detection_thread_func(self):
        """Object detection thread"""
        while self.running:
            try:
                if not self.frame_queue.empty():
                    frame = self.frame_queue.get()
                    detections = self.process_detections(frame)
                    
                    if not self.detection_queue.full():
                        self.detection_queue.put((frame, detections))
                        
            except Exception as e:
                print(f"❌ Detection error: {e}")
                time.sleep(0.1)
    
    def communication_thread_func(self):
        """ESP32 communication thread"""
        heartbeat_interval = 1.0
        last_heartbeat = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # Send heartbeat
                if current_time - last_heartbeat >= heartbeat_interval:
                    self.send_command("HEARTBEAT")
                    last_heartbeat = current_time
                
                # Process detection data
                if not self.detection_queue.empty():
                    frame, detections = self.detection_queue.get()
                    self.process_robot_behavior(detections)
                
                # Read serial data
                if self.serial_port and self.serial_port.in_waiting:
                    try:
                        response = self.serial_port.readline().decode().strip()
                        if response:
                            self.process_serial_response(response)
                    except Exception as e:
                        print(f"❌ Serial read error: {e}")
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"❌ Communication error: {e}")
                time.sleep(0.1)
    
    def process_robot_behavior(self, detections):
        """Process robot behavior based on detections"""
        if not detections:
            # No objects detected
            if self.robot_state in [RobotState.TRACKING, RobotState.ULTRASONIC_RANGE]:
                self.send_command("NO_OBJECT")
                self.robot_state = RobotState.SEARCHING
            return
        
        # Find the best detection (closest or most centered)
        best_detection = self.select_best_detection(detections)
        self.last_detection = best_detection
        self.detection_count += 1
        
        # Send detection data to ESP32
        command = f"OBJECT_DETECTED:{best_detection.center_x},{best_detection.distance:.3f}"
        self.send_command(command)
        
        # Update robot state based on detection
        if self.robot_state == RobotState.SEARCHING:
            self.robot_state = RobotState.TRACKING
        elif self.robot_state == RobotState.TRACKING and best_detection.distance <= 0.1:
            self.robot_state = RobotState.ULTRASONIC_RANGE
    
    def select_best_detection(self, detections):
        """Select the best detection based on distance and centering"""
        # Prefer objects closer to center and closer in distance
        def score_detection(det):
            center_score = 1.0 / (1.0 + abs(det.center_x - self.frame_center_x) / 100.0)
            distance_score = 1.0 / (1.0 + det.distance)
            confidence_score = det.confidence
            return center_score * distance_score * confidence_score
        
        return max(detections, key=score_detection)
    
    def process_serial_response(self, response):
        """Process responses from ESP32"""
        if "STATE:" in response:
            state_name = response.split(":")[1].strip().lower()
            for state in RobotState:
                if state.value == state_name:
                    self.robot_state = state
                    break
        elif "COLLECTION:" in response:
            print(f"🤖 {response}")
        elif "SEARCH:" in response:
            print(f"🔍 {response}")
        elif "ENV -" in response:
            print(f"🌡️  {response}")
        elif "WARNING:" in response:
            print(f"⚠️  {response}")
    
    def start_system(self):
        """Start the autonomous robot system"""
        if not self.initialize_system():
            return False
        
        print("\n🚀 Starting Advanced Autonomous Robot System...")
        print("Controls:")
        print("  SPACE - Start/Pause operation")
        print("  ESC/Q - Quit system")
        print("  R - Reset to search mode")
        print("="*60)
        
        self.running = True
        self.system_status = SystemStatus.PAUSED
        
        # Start threads
        self.camera_thread = threading.Thread(target=self.camera_capture_thread, daemon=True)
        self.detection_thread = threading.Thread(target=self.detection_thread_func, daemon=True)
        self.communication_thread = threading.Thread(target=self.communication_thread_func, daemon=True)
        
        self.camera_thread.start()
        self.detection_thread.start()
        self.communication_thread.start()
        
        # Main display loop
        self.main_loop()
        
        return True
    
    def main_loop(self):
        """Main application loop"""
        while self.running:
            try:
                # Get latest frame and detections
                if not self.detection_queue.empty():
                    frame, detections = self.detection_queue.get()
                    
                    # Draw annotations
                    annotated_frame = self.draw_annotations(frame, detections)
                    self.draw_status_info(annotated_frame)
                    
                    # Display frame
                    cv2.imshow("Advanced Autonomous Robot - Camera Feed", annotated_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord(' '):  # Space - Start/Pause
                    if self.system_status == SystemStatus.PAUSED:
                        self.system_status = SystemStatus.RUNNING
                        self.robot_state = RobotState.SEARCHING
                        self.send_command("START_SEARCH")
                        print("▶️  System STARTED - Robot searching for objects")
                    else:
                        self.system_status = SystemStatus.PAUSED
                        self.robot_state = RobotState.STOPPED
                        self.send_command("PAUSE")  # Send PAUSE command to ESP32
                        print("⏸️  System PAUSED - Motors stopped")
                
                elif key == ord('r') or key == ord('R'):  # Reset
                    self.robot_state = RobotState.SEARCHING
                    self.send_command("START_SEARCH")
                    print("🔄 Reset to search mode")
                
                elif key == 27 or key == ord('q') or key == ord('Q'):  # ESC or Q
                    print("🛑 Shutting down system...")
                    break
                
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                print("\n🛑 Keyboard interrupt received - Shutting down...")
                break
            except Exception as e:
                print(f"❌ Main loop error: {e}")
                time.sleep(0.1)
        
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("🧹 Cleaning up resources...")
        
        self.running = False
        
        # Stop all movement
        if self.serial_port:
            self.send_command("STOP")
            time.sleep(0.5)
        
        # Wait for threads to finish
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
        if self.detection_thread and self.detection_thread.is_alive():
            self.detection_thread.join(timeout=1.0)
        if self.communication_thread and self.communication_thread.is_alive():
            self.communication_thread.join(timeout=1.0)
        
        # Clean up resources
        if self.camera:
            self.camera.release()
        if self.serial_port:
            self.serial_port.close()
        
        cv2.destroyAllWindows()
        print("✅ Cleanup completed")


def main():
    """Main entry point"""
    print("🤖 Advanced Autonomous Garbage Collector Robot")
    print("="*60)
    
    try:
        robot = AdvancedAutonomousRobot()
        robot.start_system()
        
    except KeyboardInterrupt:
        print("\n🛑 System interrupted by user")
    except Exception as e:
        print(f"❌ System error: {e}")
    finally:
        print("👋 Advanced Autonomous Robot System terminated")


if __name__ == "__main__":
    main()