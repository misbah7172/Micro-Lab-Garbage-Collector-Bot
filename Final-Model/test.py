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

# Optionally, suppress the future warning from PyTorch
warnings.filterwarnings("ignore", category=FutureWarning)

def find_esp32_port():
    """Automatically find ESP32 port on Windows"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "CH340" in port.description or "CP210" in port.description or "USB" in port.description:
            return port.device
    return None

def setup_serial_connection():
    """Setup serial connection with automatic port detection"""
    # First try to find ESP32 automatically
    auto_port = find_esp32_port()
    
    if auto_port:
        print(f"Found potential ESP32 on port: {auto_port}")
        serial_port = auto_port
    else:
        # Fallback to manual configuration
        serial_port = 'COM3'  # Update this to your ESP32 serial port
        print(f"Auto-detection failed, trying manual port: {serial_port}")
    
    baud_rate = 115200  # Higher baud rate for faster communication
    
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"✓ Connected to ESP32 on {serial_port}")
        return ser
    except serial.SerialException as e:
        print(f"✗ Error connecting to serial port {serial_port}: {e}")
        print("Available ports:")
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  - {port.device}: {port.description}")
        print("\nPlease check your ESP32 connection and update the serial_port variable")
        return None

def setup_yolo_model():
    """Setup YOLO model with error handling"""
    model_path = 'best.pt'
    
    if not os.path.exists(model_path):
        print(f"✗ YOLO model not found at: {model_path}")
        print("Please ensure the model file exists")
        return None
    
    try:
        model = YOLO(model_path)
        print(f"✓ YOLO model loaded successfully")
        return model
    except Exception as e:
        print(f"✗ Error loading YOLO model: {e}")
        return None

def setup_camera():
    """Setup camera with error handling"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("✗ Unable to load camera feed")
        print("Please check if your camera is connected and not being used by another application")
        return None
    
    # Set camera properties for better performance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("✓ Camera initialized successfully")
    return cap

# Global variables for dashboard data
dashboard_data = {
    'robot_status': 'INITIALIZING',
    'movement_direction': 'STOPPED',
    'detections_count': 0,
    'total_collected': 0,
    'temperature': 0.0,
    'humidity': 0.0,
    'smoke_level': 0,
    'distance': 0,
    'last_collection_time': 'Never',
    'system_uptime': 0,
    'frame_rate': 0,
    'connection_status': 'DISCONNECTED',
    'last_command': 'None',
    'collection_in_progress': False
}

def send_heartbeat(ser):
    """Send heartbeat signal to ESP32"""
    try:
        ser.write(b'H')
    except:
        pass

def parse_sensor_data(data_line):
    """Parse sensor data from ESP32 serial output"""
    global dashboard_data
    try:
        # Parse temperature, humidity, smoke data
        if "Temp:" in data_line:
            temp_match = re.search(r'Temp: ([\d.]+)', data_line)
            humidity_match = re.search(r'Humidity: ([\d.]+)', data_line)
            smoke_match = re.search(r'Smoke: (\d+)', data_line)
            
            if temp_match:
                dashboard_data['temperature'] = float(temp_match.group(1))
            if humidity_match:
                dashboard_data['humidity'] = float(humidity_match.group(1))
            if smoke_match:
                dashboard_data['smoke_level'] = int(smoke_match.group(1))
        
        # Parse robot status messages
        if "Moving forward" in data_line:
            dashboard_data['movement_direction'] = 'FORWARD'
            dashboard_data['robot_status'] = 'MOVING'
            dashboard_data['last_command'] = 'Forward'
        elif "Rotating to search" in data_line:
            dashboard_data['movement_direction'] = 'ROTATING'
            dashboard_data['robot_status'] = 'SEARCHING'
            dashboard_data['last_command'] = 'Rotate'
        elif "Stopped" in data_line:
            dashboard_data['movement_direction'] = 'STOPPED'
            dashboard_data['robot_status'] = 'IDLE'
            dashboard_data['last_command'] = 'Stop'
        elif "Starting trash collection" in data_line:
            dashboard_data['collection_in_progress'] = True
            dashboard_data['robot_status'] = 'COLLECTING'
            dashboard_data['movement_direction'] = 'COLLECTING'
        elif "Trash collection completed" in data_line:
            dashboard_data['collection_in_progress'] = False
            dashboard_data['total_collected'] += 1
            dashboard_data['last_collection_time'] = datetime.now().strftime("%H:%M:%S")
            dashboard_data['robot_status'] = 'COMPLETED_COLLECTION'
            dashboard_data['movement_direction'] = 'STOPPED'
        
    except Exception as e:
        print(f"Error parsing sensor data: {e}")

def read_serial_data(ser):
    """Background thread to read serial data from ESP32"""
    global dashboard_data
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"ESP32: {line}")  # Debug output
                    parse_sensor_data(line)
        except Exception as e:
            print(f"Serial read error: {e}")
            break
        time.sleep(0.1)

def draw_dashboard_panel(frame, x, y, width, height, title, data_dict, colors):
    """Draw a dashboard panel with title and data"""
    # Panel background
    cv2.rectangle(frame, (x, y), (x + width, y + height), (40, 40, 40), -1)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (100, 100, 100), 2)
    
    # Title bar
    cv2.rectangle(frame, (x, y), (x + width, y + 30), (60, 60, 60), -1)
    cv2.putText(frame, title, (x + 10, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Data entries
    y_offset = 50
    for key, value in data_dict.items():
        color = colors.get(key, (255, 255, 255))
        text = f"{key}: {value}"
        cv2.putText(frame, text, (x + 10, y + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y_offset += 25

def create_dashboard_overlay(frame, detections_count):
    """Create comprehensive dashboard overlay"""
    global dashboard_data
    
    # Update frame rate and system uptime
    current_time = time.time()
    dashboard_data['frame_rate'] = round(1.0 / max(0.001, current_time - getattr(create_dashboard_overlay, 'last_time', current_time)), 1)
    create_dashboard_overlay.last_time = current_time
    
    # Calculate uptime
    if not hasattr(create_dashboard_overlay, 'start_time'):
        create_dashboard_overlay.start_time = current_time
    dashboard_data['system_uptime'] = round(current_time - create_dashboard_overlay.start_time, 1)
    
    # Update detection count
    dashboard_data['detections_count'] = detections_count
    
    # Create main dashboard background
    overlay = frame.copy()
    
    # System Status Panel (Top Left)
    system_data = {
        'Status': dashboard_data['robot_status'],
        'Direction': dashboard_data['movement_direction'],
        'Connection': dashboard_data['connection_status'],
        'Uptime': f"{dashboard_data['system_uptime']}s",
        'FPS': dashboard_data['frame_rate']
    }
    
    status_colors = {
        'Status': (0, 255, 0) if dashboard_data['robot_status'] in ['IDLE', 'MOVING', 'SEARCHING'] else (0, 255, 255),
        'Direction': (0, 255, 0) if dashboard_data['movement_direction'] != 'STOPPED' else (255, 255, 255),
        'Connection': (0, 255, 0) if dashboard_data['connection_status'] == 'CONNECTED' else (0, 0, 255)
    }
    
    draw_dashboard_panel(overlay, 10, 10, 200, 150, "SYSTEM STATUS", system_data, status_colors)
    
    # Detection & Collection Panel (Top Center)
    detection_data = {
        'Live Detections': dashboard_data['detections_count'],
        'Total Collected': dashboard_data['total_collected'],
        'Last Collection': dashboard_data['last_collection_time'],
        'Collecting': 'YES' if dashboard_data['collection_in_progress'] else 'NO',
        'Last Command': dashboard_data['last_command']
    }
    
    detection_colors = {
        'Live Detections': (0, 255, 255) if dashboard_data['detections_count'] > 0 else (255, 255, 255),
        'Total Collected': (0, 255, 0),
        'Collecting': (255, 255, 0) if dashboard_data['collection_in_progress'] else (255, 255, 255)
    }
    
    draw_dashboard_panel(overlay, 220, 10, 200, 150, "DETECTION & COLLECTION", detection_data, detection_colors)
    
    # Environmental Sensors Panel (Top Right)
    sensor_data = {
        'Temperature': f"{dashboard_data['temperature']:.1f}°C",
        'Humidity': f"{dashboard_data['humidity']:.1f}%",
        'Smoke Level': dashboard_data['smoke_level'],
        'Distance': f"{dashboard_data['distance']}cm"
    }
    
    sensor_colors = {
        'Temperature': (0, 0, 255) if dashboard_data['temperature'] > 40 else (0, 255, 0),
        'Humidity': (0, 255, 255),
        'Smoke Level': (0, 0, 255) if dashboard_data['smoke_level'] > 300 else (0, 255, 0),
        'Distance': (255, 255, 0) if dashboard_data['distance'] < 20 else (0, 255, 0)
    }
    
    draw_dashboard_panel(overlay, 430, 10, 200, 150, "ENVIRONMENTAL", sensor_data, sensor_colors)
    
    # Movement Direction Indicator (Bottom Left)
    direction_x, direction_y = 50, frame.shape[0] - 100
    cv2.circle(overlay, (direction_x, direction_y), 30, (50, 50, 50), -1)
    cv2.circle(overlay, (direction_x, direction_y), 30, (100, 100, 100), 2)
    
    # Direction arrow based on movement
    if dashboard_data['movement_direction'] == 'FORWARD':
        cv2.arrowedLine(overlay, (direction_x, direction_y + 15), (direction_x, direction_y - 15), (0, 255, 0), 3)
    elif dashboard_data['movement_direction'] == 'ROTATING':
        cv2.ellipse(overlay, (direction_x, direction_y), (15, 15), 0, 0, 270, (255, 255, 0), 3)
    elif dashboard_data['movement_direction'] == 'COLLECTING':
        cv2.circle(overlay, (direction_x, direction_y), 10, (0, 255, 255), -1)
    else:  # STOPPED
        cv2.rectangle(overlay, (direction_x - 10, direction_y - 10), (direction_x + 10, direction_y + 10), (255, 0, 0), -1)
    
    cv2.putText(overlay, dashboard_data['movement_direction'], (direction_x - 30, direction_y + 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Control Instructions (Bottom Right)
    instructions = [
        "CONTROLS:",
        "ESC - Exit",
        "S - Stop Robot", 
        "C - Collect Trash",
        "H - Send Heartbeat"
    ]
    
    start_y = frame.shape[0] - 120
    for i, instruction in enumerate(instructions):
        color = (0, 255, 255) if i == 0 else (255, 255, 255)
        cv2.putText(overlay, instruction, (frame.shape[1] - 150, start_y + i * 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    
    # Blend overlay with original frame
    alpha = 0.7
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
    
    return frame

# Initialize system components
print("=== Micro Lab Garbage Collector ===")
print("Initializing system components...")

# Set up the YOLO model
model = setup_yolo_model()
if model is None:
    sys.exit(1)

# Set up the serial connection to the ESP32
ser = setup_serial_connection()
if ser is None:
    sys.exit(1)
else:
    dashboard_data['connection_status'] = 'CONNECTED'
    # Start background thread for reading serial data
    serial_thread = threading.Thread(target=read_serial_data, args=(ser,), daemon=True)
    serial_thread.start()

# Set up camera
cap = setup_camera()
if cap is None:
    sys.exit(1)

# Initialize annotators with custom settings
bounding_box_annotator = sv.RoundBoxAnnotator(
    color=sv.Color.from_hex("#FF6B6B"),
    thickness=2,
    color_lookup=sv.ColorLookup.INDEX
)
label_annotator = sv.LabelAnnotator(
    color=sv.Color.from_hex("#FFFFFF"),
    text_color=sv.Color.from_hex("#000000"),
    text_scale=0.5,
    text_thickness=1
)

# System variables
last_heartbeat = time.time()
heartbeat_interval = 5  # Send heartbeat every 5 seconds
detection_count = 0
frame_count = 0
start_time = time.time()

dashboard_data['robot_status'] = 'READY'
dashboard_data['connection_status'] = 'CONNECTED'

print("✓ System initialized successfully!")
print("Dashboard Controls:")
print("  ESC - Exit system")
print("  S - Stop robot") 
print("  C - Force trash collection")
print("  H - Send manual heartbeat")
print("Starting AI detection with dashboard...")

# Main loop for object detection
try:
    while True:
        ret, frame = cap.read()
        frame_count += 1

        if not ret:
            print("✗ Failed to read from camera")
            break

        # Perform object detection
        results = model(frame)[0]
        detections = sv.Detections.from_ultralytics(results)

        # Annotate the detection
        annotated_image = bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)

        # Create comprehensive dashboard overlay
        dashboard_frame = create_dashboard_overlay(annotated_image, len(detections))

        # Show the camera feed with dashboard
        cv2.imshow('Micro Lab Garbage Collector - Dashboard', dashboard_frame)

        # Send commands to ESP32 based on detections
        try:
            if len(detections) > 0:
                ser.write(b'F')  # 'F' for moving forward
                detection_count += 1
                dashboard_data['last_command'] = 'Forward'
                dashboard_data['movement_direction'] = 'FORWARD'
                dashboard_data['robot_status'] = 'APPROACHING_TARGET'
                print(f"🎯 Trash detected! (Count: {detection_count}) Moving forward...")
            else:
                ser.write(b'R')  # 'R' for rotating/searching for garbage
                dashboard_data['last_command'] = 'Rotate'
                dashboard_data['movement_direction'] = 'ROTATING'
                dashboard_data['robot_status'] = 'SEARCHING'
        except serial.SerialException as e:
            print(f"❌ Serial communication error: {e}")
            dashboard_data['connection_status'] = 'ERROR'
            break

        # Send periodic heartbeat
        current_time = time.time()
        if current_time - last_heartbeat > heartbeat_interval:
            send_heartbeat(ser)
            last_heartbeat = current_time

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:  # Escape key
            print("🛑 Escape pressed - Shutting down...")
            break
        elif key == ord('s') or key == ord('S'):  # Stop command
            ser.write(b'S')
            dashboard_data['last_command'] = 'Stop'
            dashboard_data['movement_direction'] = 'STOPPED'
            dashboard_data['robot_status'] = 'STOPPED'
            print("⏹️ Stop command sent to robot")
        elif key == ord('c') or key == ord('C'):  # Collect command
            ser.write(b'C')
            dashboard_data['last_command'] = 'Collect'
            dashboard_data['movement_direction'] = 'COLLECTING'
            dashboard_data['robot_status'] = 'MANUAL_COLLECTION'
            print("🤖 Manual collect command sent to robot")
        elif key == ord('h') or key == ord('H'):  # Manual heartbeat
            send_heartbeat(ser)
            print("💓 Manual heartbeat sent")

except KeyboardInterrupt:
    print("\n⚠️ Keyboard interrupt received - Shutting down...")
except Exception as e:
    print(f"❌ Unexpected error: {e}")
finally:
    # Cleanup
    print("🧹 Cleaning up system...")
    dashboard_data['robot_status'] = 'SHUTTING_DOWN'
    dashboard_data['connection_status'] = 'DISCONNECTING'
    
    try:
        ser.write(b'S')  # Send stop command
        time.sleep(0.5)  # Give time for command to send
        ser.close()
        print("✓ Serial connection closed")
    except:
        pass
    
    cap.release()
    cv2.destroyAllWindows()
    print("✓ Camera and windows closed")
    print("✅ System shutdown complete")
    print(f"📊 Session Summary:")
    print(f"   - Total Runtime: {time.time() - start_time:.1f} seconds")
    print(f"   - Items Detected: {detection_count}")
    print(f"   - Items Collected: {dashboard_data['total_collected']}")
    print(f"   - Frames Processed: {frame_count}")
