#!/usr/bin/env python3
"""
Micro Lab Garbage Collector - Web Dashboard
Modern browser-based interface with separate panels for camera and data
"""

import os
import cv2
import serial
import serial.tools.list_ports
import supervision as sv
from ultralytics import YOLO
import warnings
import time
import sys
import threading
import re
from datetime import datetime
import numpy as np
import base64
from flask import Flask, render_template, Response, jsonify
from flask_socketio import SocketIO, emit
import json
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Suppress warnings
warnings.filterwarnings("ignore", category=FutureWarning)

# Configuration from .env file
class Config:
    # AI Model Configuration
    MODEL_PATH = os.getenv('MODEL_PATH', 'best.pt')
    MODEL_INPUT_SIZE = int(os.getenv('MODEL_INPUT_SIZE', '480'))
    MODEL_CONFIDENCE_THRESHOLD = float(os.getenv('MODEL_CONFIDENCE_THRESHOLD', '0.5'))
    BOTTLE_CLASS_ID = int(os.getenv('BOTTLE_CLASS_ID', '39'))
    
    # Camera Configuration
    CAMERA_INDEX = int(os.getenv('CAMERA_INDEX', '0'))
    CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '640'))
    CAMERA_HEIGHT = int(os.getenv('CAMERA_HEIGHT', '480'))
    CAMERA_FPS = int(os.getenv('CAMERA_FPS', '30'))
    JPEG_QUALITY = int(os.getenv('JPEG_QUALITY', '85'))
    
    # Serial Communication Configuration
    SERIAL_PORT = os.getenv('SERIAL_PORT', 'AUTO')
    SERIAL_BAUD_RATE = int(os.getenv('SERIAL_BAUD_RATE', '115200'))
    SERIAL_TIMEOUT = float(os.getenv('SERIAL_TIMEOUT', '1.0'))
    AUTO_DETECT_PORTS = os.getenv('AUTO_DETECT_PORTS', 'CH340,CP210,USB').split(',')
    
    # Web Server Configuration
    WEB_HOST = os.getenv('WEB_HOST', '0.0.0.0')
    WEB_PORT = int(os.getenv('WEB_PORT', '5000'))
    WEB_DEBUG = os.getenv('WEB_DEBUG', 'False').lower() == 'true'
    SECRET_KEY = os.getenv('SECRET_KEY', 'micro_lab_garbage_collector_2025')
    
    # System Performance
    TARGET_FPS = int(os.getenv('TARGET_FPS', '30'))
    FRAME_PROCESSING_DELAY = float(os.getenv('FRAME_PROCESSING_DELAY', '0.033'))
    SERIAL_READ_DELAY = float(os.getenv('SERIAL_READ_DELAY', '0.1'))
    
    # Robot Commands
    COMMAND_FORWARD = os.getenv('COMMAND_FORWARD', 'F')
    COMMAND_ROTATE = os.getenv('COMMAND_ROTATE', 'R')
    COMMAND_STOP = os.getenv('COMMAND_STOP', 'S')
    COMMAND_COLLECT = os.getenv('COMMAND_COLLECT', 'C')
    COMMAND_HEARTBEAT = os.getenv('COMMAND_HEARTBEAT', 'H')
    
    # Default Sensor Values
    DEFAULT_TEMPERATURE = float(os.getenv('DEFAULT_TEMPERATURE', '24.5'))
    DEFAULT_HUMIDITY = float(os.getenv('DEFAULT_HUMIDITY', '60.0'))
    DEFAULT_SMOKE_LEVEL = int(os.getenv('DEFAULT_SMOKE_LEVEL', '150'))
    DEFAULT_DISTANCE = int(os.getenv('DEFAULT_DISTANCE', '45'))
    
    # Dashboard Configuration
    DASHBOARD_TITLE = os.getenv('DASHBOARD_TITLE', 'Micro Lab Garbage Collector')
    DASHBOARD_SUBTITLE = os.getenv('DASHBOARD_SUBTITLE', 'AI-Powered Autonomous Cleaning System - Live Dashboard')
    
    # Development/Debug Settings
    MOCK_ESP32 = os.getenv('MOCK_ESP32', 'False').lower() == 'true'
    MOCK_CAMERA = os.getenv('MOCK_CAMERA', 'False').lower() == 'true'
    VERBOSE_OUTPUT = os.getenv('VERBOSE_OUTPUT', 'True').lower() == 'true'

# Flask setup
app = Flask(__name__)
app.config['SECRET_KEY'] = Config.SECRET_KEY
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables for system state
dashboard_data = {
    'robot_status': 'INITIALIZING',
    'movement_direction': 'STOPPED',
    'connection_status': 'DISCONNECTED',
    'detections_count': 0,
    'total_collected': 0,
    'last_collection_time': 'Never',
    'collection_in_progress': False,
    'last_command': 'None',
    'frame_rate': 0,
    'system_uptime': 0,
    'temperature': Config.DEFAULT_TEMPERATURE,
    'humidity': Config.DEFAULT_HUMIDITY,
    'smoke_level': Config.DEFAULT_SMOKE_LEVEL,
    'distance': Config.DEFAULT_DISTANCE,
    'camera_status': 'OFFLINE',
    'ai_model_status': 'LOADING'
}

# Global objects
ser = None
cap = None
model = None
detection_count = 0
frame_count = 0
start_time = time.time()
current_frame = None

def find_esp32_port():
    """Automatically find ESP32 port on Windows"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        for detect_string in Config.AUTO_DETECT_PORTS:
            if detect_string in port.description:
                return port.device
    return None

def setup_serial_connection():
    """Setup serial connection with automatic port detection"""
    global ser, dashboard_data
    
    if Config.MOCK_ESP32:
        print("🔧 Mock ESP32 mode enabled - skipping serial connection")
        dashboard_data['connection_status'] = 'MOCK'
        return None
    
    if Config.SERIAL_PORT == 'AUTO':
        auto_port = find_esp32_port()
        if auto_port:
            print(f"Found potential ESP32 on port: {auto_port}")
            serial_port = auto_port
        else:
            serial_port = 'COM3'  # Fallback
            print(f"Auto-detection failed, trying fallback port: {serial_port}")
    else:
        serial_port = Config.SERIAL_PORT
        print(f"Using configured serial port: {serial_port}")
    
    try:
        ser = serial.Serial(serial_port, Config.SERIAL_BAUD_RATE, timeout=Config.SERIAL_TIMEOUT)
        print(f"✓ Connected to ESP32 on {serial_port}")
        dashboard_data['connection_status'] = 'CONNECTED'
        return ser
    except serial.SerialException as e:
        print(f"✗ Error connecting to serial port {serial_port}: {e}")
        dashboard_data['connection_status'] = 'DISCONNECTED'
        return None

def load_ai_model():
    """Load YOLO AI model with enhanced security handling"""
    global model, dashboard_data
    
    dashboard_data['ai_model_status'] = 'LOADING'
    
    try:
        # Enhanced security patch for PyTorch 2.6+ weights_only restriction
        import torch
        original_load = torch.load
        def trusted_load(*args, **kwargs):
            kwargs['weights_only'] = False
            return original_load(*args, **kwargs)
        torch.load = trusted_load
        
        # Use configured model path
        if os.path.isabs(Config.MODEL_PATH):
            model_path = Config.MODEL_PATH
        else:
            model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), Config.MODEL_PATH)
        
        if not os.path.exists(model_path):
            print(f"Model file not found at: {model_path}")
            print(f"Current working directory: {os.getcwd()}")
            print(f"Script directory: {os.path.dirname(os.path.abspath(__file__))}")
            dashboard_data['ai_model_status'] = 'ERROR'
            return None
        
        model = YOLO(model_path)
        print(f"YOLO model loaded successfully from: {model_path}")
        dashboard_data['ai_model_status'] = 'READY'
        return model
        
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        dashboard_data['ai_model_status'] = 'ERROR'
        return None

def setup_camera():
    """Setup camera with error handling"""
    global cap, dashboard_data
    
    if Config.MOCK_CAMERA:
        print("🔧 Mock camera mode enabled - skipping camera setup")
        dashboard_data['camera_status'] = 'MOCK'
        return None
    
    try:
        cap = cv2.VideoCapture(Config.CAMERA_INDEX)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, Config.CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Config.CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, Config.CAMERA_FPS)
            print(f"✓ Camera initialized successfully (Index: {Config.CAMERA_INDEX}, Resolution: {Config.CAMERA_WIDTH}x{Config.CAMERA_HEIGHT})")
            dashboard_data['camera_status'] = 'ONLINE'
            return cap
        else:
            print("✗ Failed to open camera")
            dashboard_data['camera_status'] = 'ERROR'
            return None
    except Exception as e:
        print(f"Camera setup error: {e}")
        dashboard_data['camera_status'] = 'ERROR'
        return None

def serial_reader():
    """Background thread to read ESP32 data"""
    global ser, dashboard_data
    
    while ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    print(f"ESP32: {data}")
                    
                    # Parse sensor data
                    if "Temperature:" in data and "Humidity:" in data:
                        temp_match = re.search(r'Temperature: ([\d.]+)', data)
                        humid_match = re.search(r'Humidity: ([\d.]+)', data)
                        
                        if temp_match:
                            dashboard_data['temperature'] = float(temp_match.group(1))
                        if humid_match:
                            dashboard_data['humidity'] = float(humid_match.group(1))
                    
                    elif "Smoke level:" in data:
                        smoke_match = re.search(r'Smoke level: (\d+)', data)
                        if smoke_match:
                            dashboard_data['smoke_level'] = int(smoke_match.group(1))
                    
                    elif "Distance:" in data:
                        dist_match = re.search(r'Distance: (\d+)', data)
                        if dist_match:
                            dashboard_data['distance'] = int(dist_match.group(1))
                    
                    elif "Moving forward" in data:
                        dashboard_data['movement_direction'] = 'FORWARD'
                        dashboard_data['robot_status'] = 'MOVING'
                    
                    elif "Rotating" in data:
                        dashboard_data['movement_direction'] = 'ROTATING'
                        dashboard_data['robot_status'] = 'SEARCHING'
                    
                    elif "Collecting" in data:
                        dashboard_data['movement_direction'] = 'COLLECTING'
                        dashboard_data['robot_status'] = 'COLLECTING'
                        dashboard_data['collection_in_progress'] = True
                    
                    elif "Collection complete" in data:
                        dashboard_data['total_collected'] += 1
                        dashboard_data['last_collection_time'] = datetime.now().strftime("%H:%M:%S")
                        dashboard_data['collection_in_progress'] = False
                        dashboard_data['robot_status'] = 'READY'
                    
                    elif "Stopped" in data:
                        dashboard_data['movement_direction'] = 'STOPPED'
                        dashboard_data['robot_status'] = 'READY'
        
        except Exception as e:
            print(f"Serial read error: {e}")
            break
        
        time.sleep(Config.SERIAL_READ_DELAY)

def send_command(command):
    """Send command to ESP32"""
    global ser, dashboard_data
    
    if Config.MOCK_ESP32:
        dashboard_data['last_command'] = command
        if Config.VERBOSE_OUTPUT:
            print(f"Mock ESP32: Command {command} sent")
        return True
    
    if ser and ser.is_open:
        try:
            ser.write(command.encode())
            dashboard_data['last_command'] = command
            if Config.VERBOSE_OUTPUT:
                print(f"Sent command to ESP32: {command}")
            return True
        except Exception as e:
            print(f"Error sending command: {e}")
            return False
    return False

def generate_frames():
    """Generate camera frames for streaming"""
    global cap, model, dashboard_data, detection_count, frame_count, current_frame
    
    if not cap or not model:
        return
    
    while True:
        try:
            ret, frame = cap.read()
            if not ret:
                break
            
            frame_count += 1
            
            # Update system metrics
            current_time = time.time()
            dashboard_data['frame_rate'] = round(1.0 / max(0.001, current_time - getattr(generate_frames, 'last_time', current_time)), 1)
            generate_frames.last_time = current_time
            dashboard_data['system_uptime'] = round(current_time - start_time, 1)
            
            # Run AI detection
            results = model(frame, imgsz=Config.MODEL_INPUT_SIZE, verbose=False)
            detections = sv.Detections.from_ultralytics(results[0])
            
            # Show all detections for debugging (temporarily disabled bottle filtering)
            # bottle_detections = detections[detections.class_id == Config.BOTTLE_CLASS_ID]
            bottle_detections = detections  # Show all detections like test.py
            
            if len(bottle_detections) > 0:
                detection_count += len(bottle_detections)
                dashboard_data['detections_count'] = len(bottle_detections)  # Show current frame detections
                dashboard_data['total_detections'] = detection_count         # Keep running total
                
                # Send movement command
                send_command(Config.COMMAND_FORWARD)
                if Config.VERBOSE_OUTPUT:
                    print(f"Objects detected! (Current: {len(bottle_detections)}, Total: {detection_count}) Moving forward...")
            else:
                dashboard_data['detections_count'] = 0  # No detections in current frame
            
            # Annotate frame
            box_annotator = sv.BoxAnnotator()
            label_annotator = sv.LabelAnnotator()
            
            annotated_frame = box_annotator.annotate(scene=frame, detections=bottle_detections)
            annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=bottle_detections)
            
            # Convert frame to base64 for web streaming
            _, buffer = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, Config.JPEG_QUALITY])
            current_frame = base64.b64encode(buffer).decode('utf-8')
            
            # Ensure dashboard data consistency before emission
            dashboard_data['current_detections'] = len(bottle_detections)  # Add explicit current count
            dashboard_data['robot_status'] = 'DETECTING' if len(bottle_detections) > 0 else 'SCANNING'
            
            # Emit data to web clients with error handling
            try:
                socketio.emit('frame_update', {'frame': current_frame})
                socketio.emit('data_update', dashboard_data)
            except Exception as emit_error:
                print(f"WebSocket emission error: {emit_error}")
            
            time.sleep(Config.FRAME_PROCESSING_DELAY)
            
        except Exception as e:
            print(f"Frame processing error: {e}")
            break

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/data')
def get_data():
    """API endpoint for dashboard data"""
    return jsonify(dashboard_data)

@app.route('/api/config')
def get_config():
    """API endpoint for system configuration"""
    config_data = {
        'model_path': Config.MODEL_PATH,
        'camera_index': Config.CAMERA_INDEX,
        'camera_resolution': f"{Config.CAMERA_WIDTH}x{Config.CAMERA_HEIGHT}",
        'serial_port': Config.SERIAL_PORT,
        'serial_baud': Config.SERIAL_BAUD_RATE,
        'commands': {
            'forward': Config.COMMAND_FORWARD,
            'rotate': Config.COMMAND_ROTATE,
            'stop': Config.COMMAND_STOP,
            'collect': Config.COMMAND_COLLECT,
            'heartbeat': Config.COMMAND_HEARTBEAT
        },
        'mock_mode': {
            'esp32': Config.MOCK_ESP32,
            'camera': Config.MOCK_CAMERA
        }
    }
    return jsonify(config_data)

@app.route('/api/command/<command>')
def send_command_api(command):
    """API endpoint for sending commands"""
    valid_commands = [Config.COMMAND_FORWARD, Config.COMMAND_ROTATE, Config.COMMAND_STOP, 
                     Config.COMMAND_COLLECT, Config.COMMAND_HEARTBEAT]
    
    if command in valid_commands:
        success = send_command(command)
        return jsonify({'success': success, 'command': command})
    return jsonify({'success': False, 'error': 'Invalid command'})

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print('Client connected')
    emit('data_update', dashboard_data)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print('Client disconnected')

@socketio.on('send_command')
def handle_command(data):
    """Handle command from web interface"""
    command = data.get('command')
    valid_commands = [Config.COMMAND_FORWARD, Config.COMMAND_ROTATE, Config.COMMAND_STOP, 
                     Config.COMMAND_COLLECT, Config.COMMAND_HEARTBEAT]
    
    if command in valid_commands:
        success = send_command(command)
        emit('command_response', {'success': success, 'command': command})

def initialize_system():
    """Initialize all system components"""
    global ser, cap, model, dashboard_data
    
    print(f"🚀 Initializing {Config.DASHBOARD_TITLE}...")
    print(f"📋 Configuration loaded from .env file")
    print(f"   - Model: {Config.MODEL_PATH}")
    print(f"   - Camera: Index {Config.CAMERA_INDEX} ({Config.CAMERA_WIDTH}x{Config.CAMERA_HEIGHT})")
    print(f"   - Serial: {Config.SERIAL_PORT} @ {Config.SERIAL_BAUD_RATE} baud")
    print(f"   - Web Server: {Config.WEB_HOST}:{Config.WEB_PORT}")
    
    # Load AI model
    print("Loading AI model...")
    model = load_ai_model()
    
    # Setup camera
    print("Setting up camera...")
    cap = setup_camera()
    
    # Setup serial connection
    print("Setting up ESP32 connection...")
    ser = setup_serial_connection()
    
    # Start serial reader thread
    if ser and not Config.MOCK_ESP32:
        serial_thread = threading.Thread(target=serial_reader, daemon=True)
        serial_thread.start()
    
    dashboard_data['robot_status'] = 'READY'
    print("✓ System initialization complete!")
    print("🌐 Starting web server...")

if __name__ == '__main__':
    try:
        # Initialize system
        initialize_system()
        
        # Start camera processing in background
        if (cap and model) or (Config.MOCK_CAMERA and model):
            camera_thread = threading.Thread(target=generate_frames, daemon=True)
            camera_thread.start()
        
        print(f"📱 Dashboard available at: http://localhost:{Config.WEB_PORT}")
        print(f"🌍 Also available at: http://{Config.WEB_HOST}:{Config.WEB_PORT}")
        print("Press Ctrl+C to stop the system")
        
        # Start Flask web server
        socketio.run(app, host=Config.WEB_HOST, port=Config.WEB_PORT, debug=Config.WEB_DEBUG)
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received - Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        print("Cleaning up system...")
        if ser and ser.is_open and not Config.MOCK_ESP32:
            try:
                ser.write(Config.COMMAND_STOP.encode())  # Send stop command
                time.sleep(0.5)
                ser.close()
                print("✓ Serial connection closed")
            except:
                pass
        
        if cap and not Config.MOCK_CAMERA:
            cap.release()
            print("✓ Camera released")
        
        print("System shutdown complete")