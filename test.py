import os
import cv2
import serial
import serial.tools.list_ports
import supervision as sv
from ultralytics import YOLO
import warnings
import time
import sys

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
    model_path = 'C:/HARDARE/MicroLabGarbageCollector/best.pt'
    
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

def send_heartbeat(ser):
    """Send heartbeat signal to ESP32"""
    try:
        ser.write(b'H')
    except:
        pass

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

# Set up camera
cap = setup_camera()
if cap is None:
    sys.exit(1)

# Initialize annotators
bounding_box_annotator = sv.RoundBoxAnnotator()
label_annotator = sv.LabelAnnotator()

# System variables
last_heartbeat = time.time()
heartbeat_interval = 5  # Send heartbeat every 5 seconds
detection_count = 0
frame_count = 0

print("✓ System initialized successfully!")
print("Press 'ESC' to exit, 'S' to stop robot")
print("Starting object detection...")

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

        # Add system status overlay
        cv2.putText(annotated_image, f"Frame: {frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated_image, f"Detections: {len(detections)}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Show the camera feed with annotations
        cv2.imshow('Micro Lab Garbage Collector', annotated_image)

        # Send commands to ESP32 based on detections
        try:
            if len(detections) > 0:
                ser.write(b'F')  # 'F' for moving forward
                detection_count += 1
                print(f"Trash detected! (Count: {detection_count}) Moving forward...")
            else:
                ser.write(b'R')  # 'R' for rotating/searching for garbage
        except serial.SerialException as e:
            print(f"Serial communication error: {e}")
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
            print("Stop command sent to robot")
        elif key == ord('c') or key == ord('C'):  # Collect command
            ser.write(b'C')
            print("Collect command sent to robot")

except KeyboardInterrupt:
    print("\nKeyboard interrupt received - Shutting down...")
except Exception as e:
    print(f"Unexpected error: {e}")
finally:
    # Cleanup
    print("Cleaning up...")
    try:
        ser.write(b'S')  # Send stop command
        ser.close()
        print("✓ Serial connection closed")
    except:
        pass
    
    cap.release()
    cv2.destroyAllWindows()
    print("✓ Camera and windows closed")
    print("System shutdown complete")
