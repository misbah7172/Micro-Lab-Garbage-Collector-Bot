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
import requests
from urllib.request import urlopen

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
    'smoke_detected': False,
    'metal_detected': False,
    'distance': Config.DEFAULT_DISTANCE,
    'camera_status': 'OFFLINE',
    'ai_model_status': 'ROBOT_HANDLED',
    'last_sensor_update': 'Never'
}

# Global objects
ser = None
cap = None
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

def setup_camera():
    """Setup camera - now uses robot's video stream instead of direct camera access"""
    global dashboard_data
    
    print("🔧 Camera setup - using robot's video stream from http://127.0.0.1:5001/video_feed")
    dashboard_data['camera_status'] = 'STREAM_MODE'
    return None  # No direct camera needed

def serial_reader():
    """Background thread to read ESP32 data"""
    global ser, dashboard_data
    
    while ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    print(f"ESP32: {data}")
                    
                    # Parse structured sensor data: SENSOR_DATA:temp,humidity,smoke_level,smoke_detected,metal_detected
                    if "SENSOR_DATA:" in data:
                        try:
                            data_part = data.split(":")[1].strip()
                            values = data_part.split(",")
                            if len(values) >= 5:
                                dashboard_data['temperature'] = float(values[0])
                                dashboard_data['humidity'] = float(values[1])
                                dashboard_data['smoke_level'] = int(values[2])
                                dashboard_data['smoke_detected'] = bool(int(values[3]))
                                dashboard_data['metal_detected'] = bool(int(values[4]))
                                dashboard_data['last_sensor_update'] = time.strftime("%H:%M:%S")
                                
                                # Emit real-time updates to web clients
                                socketio.emit('sensor_update', {
                                    'temperature': dashboard_data['temperature'],
                                    'humidity': dashboard_data['humidity'],
                                    'smoke_level': dashboard_data['smoke_level'],
                                    'smoke_detected': dashboard_data['smoke_detected'],
                                    'metal_detected': dashboard_data['metal_detected']
                                })
                                
                        except (ValueError, IndexError) as e:
                            print(f"Error parsing sensor data: {e}")
                    
                    # Parse legacy sensor data (backward compatibility)
                    elif "Temperature:" in data and "Humidity:" in data:
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
    """Send command to ESP32 via robot"""
    global dashboard_data
    
    if Config.MOCK_ESP32:
        dashboard_data['last_command'] = command
        if Config.VERBOSE_OUTPUT:
            print(f"Mock ESP32: Command {command} sent")
        return True
    
    # Send command to robot, which will forward it to ESP32
    try:
        robot_command_url = f"http://127.0.0.1:5001/send_command/{command}"
        response = requests.get(robot_command_url, timeout=1.0)
        if response.status_code == 200:
            dashboard_data['last_command'] = command
            if Config.VERBOSE_OUTPUT:
                print(f"Sent command via robot: {command}")
            return True
        else:
            print(f"Robot command failed with status: {response.status_code}")
            return False
    except (requests.exceptions.RequestException, requests.exceptions.Timeout):
        if Config.VERBOSE_OUTPUT:
            print(f"Error sending command to robot: Command {command} - Robot offline")
        return False

def generate_frames():
    """Generate camera frames from robot's video stream"""
    global dashboard_data, detection_count, frame_count, current_frame
    
    robot_stream_url = "http://127.0.0.1:5001/video_feed"
    robot_sensor_url = "http://127.0.0.1:5001/sensor_data"
    
    while True:
        try:
            # Get sensor data from robot
            try:
                sensor_response = requests.get(robot_sensor_url, timeout=0.5)
                if sensor_response.status_code == 200:
                    sensor_data = sensor_response.json()
                    dashboard_data['temperature'] = sensor_data.get('temperature', 0.0)
                    dashboard_data['humidity'] = sensor_data.get('humidity', 0.0)
                    dashboard_data['smoke_level'] = sensor_data.get('smoke_level', 0)
                    dashboard_data['smoke_detected'] = sensor_data.get('smoke_detected', False)
                    dashboard_data['metal_detected'] = sensor_data.get('metal_detected', False)
                    
                    # Convert timestamp to readable time format
                    last_update_timestamp = sensor_data.get('last_sensor_update', 0)
                    if last_update_timestamp > 0:
                        dashboard_data['last_sensor_update'] = time.strftime("%H:%M:%S", time.localtime(last_update_timestamp))
                    else:
                        dashboard_data['last_sensor_update'] = "Never"
                        
                    # Debug: Print sensor data to terminal
                    print(f"📊 Sensor Data: T={dashboard_data['temperature']:.1f}°C, H={dashboard_data['humidity']:.1f}%, S={dashboard_data['smoke_level']}, Update={dashboard_data['last_sensor_update']}")
                    
                    # Emit real-time sensor updates to web clients
                    socketio.emit('sensor_update', {
                        'temperature': dashboard_data['temperature'],
                        'humidity': dashboard_data['humidity'],
                        'smoke_level': dashboard_data['smoke_level'],
                        'smoke_detected': dashboard_data['smoke_detected'],
                        'metal_detected': dashboard_data['metal_detected'],
                        'last_sensor_update': dashboard_data['last_sensor_update']
                    })
            except (requests.exceptions.RequestException, requests.exceptions.Timeout):
                pass  # Continue without sensor data if robot is not available
            
            # Get frame from robot's video stream
            try:
                response = requests.get(robot_stream_url, stream=True, timeout=1.0)
                if response.status_code == 200:
                    # Read multipart stream
                    bytes_data = b''
                    for chunk in response.iter_content(chunk_size=1024):
                        bytes_data += chunk
                        # Look for JPEG image boundaries
                        start_marker = bytes_data.find(b'\xff\xd8')
                        end_marker = bytes_data.find(b'\xff\xd9')
                        
                        if start_marker != -1 and end_marker != -1 and end_marker > start_marker:
                            # Extract JPEG image
                            jpg_data = bytes_data[start_marker:end_marker+2]
                            bytes_data = bytes_data[end_marker+2:]
                            
                            # Decode image
                            nparr = np.frombuffer(jpg_data, np.uint8)
                            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                            
                            if frame is not None:
                                frame_count += 1
                                
                                # Update system metrics
                                current_time = time.time()
                                dashboard_data['frame_rate'] = round(1.0 / max(0.001, current_time - getattr(generate_frames, 'last_time', current_time)), 1)
                                generate_frames.last_time = current_time
                                dashboard_data['system_uptime'] = round(current_time - start_time, 1)
                                
                                # Convert frame to base64 for web streaming
                                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, Config.JPEG_QUALITY])
                                current_frame = base64.b64encode(buffer).decode('utf-8')
                                
                                # Update dashboard data
                                dashboard_data['robot_status'] = 'CONNECTED'
                                dashboard_data['camera_status'] = 'STREAMING'
                                
                                # Emit data to web clients
                                try:
                                    socketio.emit('frame_update', {'frame': current_frame})
                                    socketio.emit('data_update', dashboard_data)
                                except Exception as emit_error:
                                    print(f"WebSocket emission error: {emit_error}")
                                
                                break  # Process one frame at a time
                            
                        if len(bytes_data) > 100000:  # Prevent buffer overflow
                            bytes_data = b''
                            
                else:
                    # Robot stream not available
                    dashboard_data['robot_status'] = 'DISCONNECTED'
                    dashboard_data['camera_status'] = 'OFFLINE'
                    
            except (requests.exceptions.RequestException, requests.exceptions.Timeout):
                # Create a message frame when robot is not available
                message_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(message_frame, "Robot Camera Offline", (180, 220), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(message_frame, "Start advanced_autonomous_robot.py", (120, 260), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                _, buffer = cv2.imencode('.jpg', message_frame)
                current_frame = base64.b64encode(buffer).decode('utf-8')
                
                dashboard_data['robot_status'] = 'DISCONNECTED'
                dashboard_data['camera_status'] = 'OFFLINE'
                
                try:
                    socketio.emit('frame_update', {'frame': current_frame})
                    socketio.emit('data_update', dashboard_data)
                except Exception as emit_error:
                    print(f"WebSocket emission error: {emit_error}")
            
            time.sleep(0.033)  # ~30 FPS
            
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
                     Config.COMMAND_COLLECT, Config.COMMAND_HEARTBEAT, 'ENV_TEST', 'ULTRASONIC_TEST', 'MOTOR_TEST']
    
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
    global ser, cap, dashboard_data
    
    print(f"🚀 Initializing {Config.DASHBOARD_TITLE}...")
    print(f"📋 Configuration loaded from .env file")
    print(f"   - Model: AI handled by Robot")
    print(f"   - Camera: Streaming from Robot")
    print(f"   - Sensors: Data from Robot")
    print(f"   - Web Server: {Config.WEB_HOST}:{Config.WEB_PORT}")
    
    # Setup camera (now uses robot stream)
    print("Setting up camera stream...")
    cap = setup_camera()
    
    # Note: ESP32 connection is handled by the robot
    print("ESP32 connection managed by robot...")
    
    dashboard_data['robot_status'] = 'READY'
    print("✓ System initialization complete!")
    print("🌐 Starting web server...")

if __name__ == '__main__':
    try:
        # Initialize system
        initialize_system()
        
        # Start camera stream processing in background
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
        
        print("System shutdown complete")