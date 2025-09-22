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

# Suppress warnings
warnings.filterwarnings("ignore", category=FutureWarning)

# Flask setup
app = Flask(__name__)
app.config['SECRET_KEY'] = 'micro_lab_garbage_collector_2025'
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
    'temperature': 24.5,
    'humidity': 60.0,
    'smoke_level': 150,
    'distance': 45,
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
        if "CH340" in port.description or "CP210" in port.description or "USB" in port.description:
            return port.device
    return None

def setup_serial_connection():
    """Setup serial connection with automatic port detection"""
    global ser, dashboard_data
    
    auto_port = find_esp32_port()
    
    if auto_port:
        print(f"Found potential ESP32 on port: {auto_port}")
        serial_port = auto_port
    else:
        serial_port = 'COM3'
        print(f"Auto-detection failed, trying manual port: {serial_port}")
    
    baud_rate = 115200
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
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
        
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'best.pt')
        
        if not os.path.exists(model_path):
            print(f"Model file not found at: {model_path}")
            dashboard_data['ai_model_status'] = 'ERROR'
            return None
        
        model = YOLO(model_path)
        print("YOLO model loaded successfully (trusted source method)")
        dashboard_data['ai_model_status'] = 'READY'
        return model
        
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        dashboard_data['ai_model_status'] = 'ERROR'
        return None

def setup_camera():
    """Setup camera with error handling"""
    global cap, dashboard_data
    
    try:
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            print("✓ Camera initialized successfully")
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
        
        time.sleep(0.1)

def send_command(command):
    """Send command to ESP32"""
    global ser, dashboard_data
    
    if ser and ser.is_open:
        try:
            ser.write(command.encode())
            dashboard_data['last_command'] = command
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
            results = model(frame, imgsz=480, verbose=False)
            detections = sv.Detections.from_ultralytics(results[0])
            
            # Filter for bottles only
            bottle_detections = detections[detections.class_id == 39]  # 39 is bottle class in COCO
            
            if len(bottle_detections) > 0:
                detection_count += len(bottle_detections)
                dashboard_data['detections_count'] = detection_count
                
                # Send movement command
                send_command('F')  # Forward
                print(f"Trash detected! (Count: {detection_count}) Moving forward...")
            
            # Annotate frame
            box_annotator = sv.BoxAnnotator()
            label_annotator = sv.LabelAnnotator()
            
            annotated_frame = box_annotator.annotate(scene=frame, detections=bottle_detections)
            annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=bottle_detections)
            
            # Convert frame to base64 for web streaming
            _, buffer = cv2.imencode('.jpg', annotated_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            current_frame = base64.b64encode(buffer).decode('utf-8')
            
            # Emit data to web clients
            socketio.emit('frame_update', {'frame': current_frame})
            socketio.emit('data_update', dashboard_data)
            
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

@app.route('/api/command/<command>')
def send_command_api(command):
    """API endpoint for sending commands"""
    if command in ['F', 'R', 'S', 'C', 'H']:
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
    if command in ['F', 'R', 'S', 'C', 'H']:
        success = send_command(command)
        emit('command_response', {'success': success, 'command': command})

def initialize_system():
    """Initialize all system components"""
    global ser, cap, model, dashboard_data
    
    print("🚀 Initializing Micro Lab Garbage Collector Web Dashboard...")
    
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
    if ser:
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
        if cap and model:
            camera_thread = threading.Thread(target=generate_frames, daemon=True)
            camera_thread.start()
        
        print("📱 Dashboard available at: http://localhost:5000")
        print("Press Ctrl+C to stop the system")
        
        # Start Flask web server
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
        
    except KeyboardInterrupt:
        print("\n🛑 Keyboard interrupt received - Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        print("Cleaning up system...")
        if ser and ser.is_open:
            try:
                ser.write(b'S')  # Send stop command
                time.sleep(0.5)
                ser.close()
                print("✓ Serial connection closed")
            except:
                pass
        
        if cap:
            cap.release()
            print("✓ Camera released")
        
        print("System shutdown complete")