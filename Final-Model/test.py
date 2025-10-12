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
        # Handle PyTorch 2.6+ weights_only security issue
        import torch
        
        # Method 1: Try with specific safe globals
        try:
            # Add safe globals for YOLO model components
            import ultralytics.nn.tasks
            torch.serialization.add_safe_globals([ultralytics.nn.tasks.DetectionModel])
            model = YOLO(model_path)
            print(f"✓ YOLO model loaded successfully")
            return model
        except:
            pass
        
        # Method 2: Try loading with weights_only=False (if trusted source)
        try:
            # Monkey patch torch.load temporarily for YOLO
            original_load = torch.load
            def patched_load(*args, **kwargs):
                kwargs['weights_only'] = False
                return original_load(*args, **kwargs)
            
            torch.load = patched_load
            model = YOLO(model_path)
            torch.load = original_load  # Restore original
            print(f"✓ YOLO model loaded successfully (trusted source method)")
            return model
        except Exception as e3:
            torch.load = original_load  # Restore original
            raise e3
            
    except Exception as e:
        print(f"✗ Error loading YOLO model: {e}")
        print("Note: This is likely due to PyTorch 2.6+ security restrictions")
        print("Suggestion: Try using a newer version of ultralytics or retrain the model")
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

def draw_modern_panel(frame, x, y, width, height, title, data_dict, colors, icon=None):
    """Draw a modern dashboard panel with gradient background and clean typography"""
    # Main panel background with rounded corners effect
    cv2.rectangle(frame, (x, y), (x + width, y + height), (30, 30, 30), -1)
    cv2.rectangle(frame, (x, y), (x + width, y + height), (80, 80, 80), 1)
    
    # Header section with accent color
    header_height = 35
    cv2.rectangle(frame, (x, y), (x + width, y + header_height), (45, 45, 45), -1)
    cv2.rectangle(frame, (x, y), (x + width, y + header_height), (100, 100, 100), 1)
    
    # Title with icon
    title_text = f"{icon} {title}" if icon else title
    cv2.putText(frame, title_text, (x + 12, y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # Data entries with proper spacing
    y_offset = header_height + 20
    line_height = 22
    
    for i, (key, value) in enumerate(data_dict.items()):
        color = colors.get(key, (200, 200, 200))
        
        # Key label
        cv2.putText(frame, f"{key}:", (x + 12, y + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        
        # Value with color coding
        value_text = str(value)
        cv2.putText(frame, value_text, (x + 85, y + y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        
        y_offset += line_height

def draw_status_indicator(frame, x, y, status, label):
    """Draw a modern status indicator with colored dot and label"""
    # Status colors
    status_colors = {
        'CONNECTED': (0, 255, 0),
        'ONLINE': (0, 255, 0),
        'ACTIVE': (0, 255, 0),
        'READY': (0, 255, 0),
        'MOVING': (255, 165, 0),
        'SEARCHING': (255, 255, 0),
        'COLLECTING': (0, 255, 255),
        'DISCONNECTED': (0, 0, 255),
        'OFFLINE': (0, 0, 255),
        'ERROR': (0, 0, 255),
        'STOPPED': (128, 128, 128)
    }
    
    color = status_colors.get(status, (128, 128, 128))
    
    # Status dot
    cv2.circle(frame, (x + 8, y + 8), 5, color, -1)
    cv2.circle(frame, (x + 8, y + 8), 5, (255, 255, 255), 1)
    
    # Label
    cv2.putText(frame, label, (x + 20, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # Status text
    cv2.putText(frame, status, (x + 20, y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1)

def draw_progress_bar(frame, x, y, width, value, max_value, label, color=(0, 255, 0)):
    """Draw a modern progress bar"""
    bar_height = 12
    
    # Background
    cv2.rectangle(frame, (x, y), (x + width, y + bar_height), (50, 50, 50), -1)
    cv2.rectangle(frame, (x, y), (x + width, y + bar_height), (100, 100, 100), 1)
    
    # Progress fill
    progress_width = int((value / max_value) * width) if max_value > 0 else 0
    if progress_width > 0:
        cv2.rectangle(frame, (x + 1, y + 1), (x + progress_width - 1, y + bar_height - 1), color, -1)
    
    # Label and value
    label_text = f"{label}: {value}"
    cv2.putText(frame, label_text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

def create_modern_dashboard_overlay(frame, detections_count):
    """Create a modern, modular dashboard overlay"""
    global dashboard_data
    
    # Update system metrics
    current_time = time.time()
    dashboard_data['frame_rate'] = round(1.0 / max(0.001, current_time - getattr(create_modern_dashboard_overlay, 'last_time', current_time)), 1)
    create_modern_dashboard_overlay.last_time = current_time
    
    if not hasattr(create_modern_dashboard_overlay, 'start_time'):
        create_modern_dashboard_overlay.start_time = current_time
    dashboard_data['system_uptime'] = round(current_time - create_modern_dashboard_overlay.start_time, 1)
    dashboard_data['detections_count'] = detections_count
    
    # Create overlay
    overlay = frame.copy()
    h, w = frame.shape[:2]
    
    # Modern dark overlay background
    cv2.rectangle(overlay, (0, 0), (w, 180), (20, 20, 20), -1)
    cv2.rectangle(overlay, (0, h-120), (w, h), (20, 20, 20), -1)
    
    # === TOP PANEL SECTION ===
    panel_width = 180
    panel_height = 160
    margin = 10
    
    # 1. SYSTEM STATUS PANEL (Top Left)
    system_data = {
        'Status': dashboard_data['robot_status'],
        'Uptime': f"{dashboard_data['system_uptime']}s",
        'FPS': f"{dashboard_data['frame_rate']}",
        'Commands': dashboard_data['last_command']
    }
    
    status_colors = {
        'Status': (0, 255, 0) if dashboard_data['robot_status'] in ['READY', 'MOVING', 'SEARCHING'] else (255, 165, 0),
        'Uptime': (255, 255, 255),
        'FPS': (0, 255, 255),
        'Commands': (255, 255, 0)
    }
    
    draw_modern_panel(overlay, margin, margin, panel_width, panel_height, "SYSTEM", system_data, status_colors, "⚙️")
    
    # 2. DETECTION PANEL (Top Center-Left)
    detection_data = {
        'Live': dashboard_data['detections_count'],
        'Collected': dashboard_data['total_collected'],
        'Last Time': dashboard_data['last_collection_time'],
        'In Progress': 'YES' if dashboard_data['collection_in_progress'] else 'NO'
    }
    
    detection_colors = {
        'Live': (0, 255, 255) if dashboard_data['detections_count'] > 0 else (128, 128, 128),
        'Collected': (0, 255, 0),
        'Last Time': (255, 255, 255),
        'In Progress': (255, 255, 0) if dashboard_data['collection_in_progress'] else (128, 128, 128)
    }
    
    draw_modern_panel(overlay, margin + panel_width + margin, margin, panel_width, panel_height, "DETECTION", detection_data, detection_colors, "🎯")
    
    # 3. ENVIRONMENTAL PANEL (Top Center-Right)
    env_data = {
        'Temp': f"{dashboard_data['temperature']:.1f}°C",
        'Humidity': f"{dashboard_data['humidity']:.1f}%",
        'Smoke': f"{dashboard_data['smoke_level']}",
        'Distance': f"{dashboard_data['distance']}cm"
    }
    
    env_colors = {
        'Temp': (0, 0, 255) if dashboard_data['temperature'] > 40 else (0, 255, 0),
        'Humidity': (0, 255, 255),
        'Smoke': (255, 0, 0) if dashboard_data['smoke_level'] > 300 else (0, 255, 0),
        'Distance': (255, 165, 0) if dashboard_data['distance'] < 20 else (0, 255, 0)
    }
    
    draw_modern_panel(overlay, margin + 2*(panel_width + margin), margin, panel_width, panel_height, "SENSORS", env_data, env_colors, "🌡️")
    
    # 4. CONNECTION STATUS PANEL (Top Right)
    conn_data = {
        'Serial': dashboard_data['connection_status'],
        'Direction': dashboard_data['movement_direction'],
        'Mode': 'AUTO' if dashboard_data['detections_count'] > 0 else 'SEARCH',
        'Battery': 'Good'  # Placeholder
    }
    
    conn_colors = {
        'Serial': (0, 255, 0) if dashboard_data['connection_status'] == 'CONNECTED' else (255, 0, 0),
        'Direction': (0, 255, 0) if dashboard_data['movement_direction'] != 'STOPPED' else (128, 128, 128),
        'Mode': (255, 255, 0),
        'Battery': (0, 255, 0)
    }
    
    draw_modern_panel(overlay, margin + 3*(panel_width + margin), margin, panel_width, panel_height, "CONNECTION", conn_data, conn_colors, "📡")
    
    # === CAMERA FEED BORDER ===
    camera_area_x = margin
    camera_area_y = panel_height + 2*margin
    camera_area_w = w - 2*margin
    camera_area_h = h - panel_height - 140
    
    # Camera feed border
    cv2.rectangle(overlay, (camera_area_x, camera_area_y), (camera_area_x + camera_area_w, camera_area_y + camera_area_h), (80, 80, 80), 2)
    
    # Camera feed title
    cv2.putText(overlay, "🎥 LIVE CAMERA FEED", (camera_area_x + 10, camera_area_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    
    # === BOTTOM PANEL SECTION ===
    bottom_y = h - 110
    
    # Movement Direction Indicator (Bottom Left)
    movement_panel_x = margin
    movement_panel_y = bottom_y
    movement_panel_w = 200
    movement_panel_h = 100
    
    cv2.rectangle(overlay, (movement_panel_x, movement_panel_y), (movement_panel_x + movement_panel_w, movement_panel_y + movement_panel_h), (30, 30, 30), -1)
    cv2.rectangle(overlay, (movement_panel_x, movement_panel_y), (movement_panel_x + movement_panel_w, movement_panel_y + movement_panel_h), (80, 80, 80), 1)
    
    # Movement title
    cv2.putText(overlay, "🧭 MOVEMENT", (movement_panel_x + 10, movement_panel_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Direction indicator
    indicator_x = movement_panel_x + 100
    indicator_y = movement_panel_y + 60
    
    if dashboard_data['movement_direction'] == 'FORWARD':
        cv2.arrowedLine(overlay, (indicator_x, indicator_y + 15), (indicator_x, indicator_y - 15), (0, 255, 0), 3)
        direction_text = "FORWARD"
        direction_color = (0, 255, 0)
    elif dashboard_data['movement_direction'] == 'ROTATING':
        cv2.ellipse(overlay, (indicator_x, indicator_y), (15, 15), 0, 0, 270, (255, 255, 0), 3)
        direction_text = "ROTATING"
        direction_color = (255, 255, 0)
    elif dashboard_data['movement_direction'] == 'COLLECTING':
        cv2.circle(overlay, (indicator_x, indicator_y), 12, (0, 255, 255), -1)
        direction_text = "COLLECTING"
        direction_color = (0, 255, 255)
    else:
        cv2.rectangle(overlay, (indicator_x - 12, indicator_y - 12), (indicator_x + 12, indicator_y + 12), (255, 0, 0), -1)
        direction_text = "STOPPED"
        direction_color = (255, 0, 0)
    
    cv2.putText(overlay, direction_text, (movement_panel_x + 10, movement_panel_y + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.4, direction_color, 1)
    
    # Status Indicators (Bottom Center)
    status_panel_x = movement_panel_x + movement_panel_w + margin
    status_indicators = [
        (dashboard_data['connection_status'], 'Serial Connection'),
        (dashboard_data['robot_status'], 'Robot Status'),
        ('ACTIVE', 'AI Detection')
    ]
    
    for i, (status, label) in enumerate(status_indicators):
        draw_status_indicator(overlay, status_panel_x + i*150, bottom_y + 20, status, label)
    
    # Control Instructions (Bottom Right)
    controls_x = w - 200
    controls_y = bottom_y
    controls_data = [
        "CONTROLS:",
        "ESC - Exit System",
        "S - Stop Robot", 
        "C - Collect Trash",
        "H - Heartbeat"
    ]
    
    cv2.rectangle(overlay, (controls_x, controls_y), (controls_x + 190, controls_y + 100), (30, 30, 30), -1)
    cv2.rectangle(overlay, (controls_x, controls_y), (controls_x + 190, controls_y + 100), (80, 80, 80), 1)
    
    for i, instruction in enumerate(controls_data):
        color = (0, 255, 255) if i == 0 else (255, 255, 255)
        weight = 1 if i == 0 else 1
        cv2.putText(overlay, instruction, (controls_x + 10, controls_y + 20 + i * 16), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, weight)
    
    # Performance metrics (Top of camera area)
    metrics_y = camera_area_y + 20
    cv2.putText(overlay, f"Detection Count: {detections_count} | Processing: {dashboard_data['frame_rate']} FPS | Uptime: {dashboard_data['system_uptime']}s", 
                (camera_area_x + 10, metrics_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Blend overlay with original frame
    alpha = 0.85
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
        dashboard_frame = create_modern_dashboard_overlay(annotated_image, len(detections))

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
                print(f"Trash detected! (Count: {detection_count}) Moving forward...")
            else:
                ser.write(b'R')  # 'R' for rotating/searching for garbage
                dashboard_data['last_command'] = 'Rotate'
                dashboard_data['movement_direction'] = 'ROTATING'
                dashboard_data['robot_status'] = 'SEARCHING'
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
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
            print("Escape pressed - Shutting down...")
            break
        elif key == ord('s') or key == ord('S'):  # Stop command
            ser.write(b'S')
            dashboard_data['last_command'] = 'Stop'
            dashboard_data['movement_direction'] = 'STOPPED'
            dashboard_data['robot_status'] = 'STOPPED'
            print("Stop command sent to robot")
        elif key == ord('c') or key == ord('C'):  # Collect command
            ser.write(b'C')
            dashboard_data['last_command'] = 'Collect'
            dashboard_data['movement_direction'] = 'COLLECTING'
            dashboard_data['robot_status'] = 'MANUAL_COLLECTION'
            print("Manual collect command sent to robot")
        elif key == ord('h') or key == ord('H'):  # Manual heartbeat
            send_heartbeat(ser)
            print("Manual heartbeat sent")

except KeyboardInterrupt:
    print("\n Keyboard interrupt received - Shutting down...")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    # Cleanup
    print("Cleaning up system...")
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
    print("Camera and windows closed")
    print("System shutdown complete")
    print(f"Session Summary:")
    print(f"   - Total Runtime: {time.time() - start_time:.1f} seconds")
    print(f"   - Items Detected: {detection_count}")
    print(f"   - Items Collected: {dashboard_data['total_collected']}")
    print(f"   - Frames Processed: {frame_count}")
