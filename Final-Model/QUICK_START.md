#  Quick Model Switching Examples

## Example 1: Switch to different model file
```bash
# Run the configuration manager
python config_manager.py

# Select option 2 (Change model path)
# Choose from the available models:
# 1. C:\HARDARE\MicroLabGarbageCollector\Final-Model\best.pt      (Current model)
# 2. C:\HARDARE\MicroLabGarbageCollector\Test-Model\best.pt       (Test model)
# 3. C:\HARDARE\MicroLabGarbageCollector\Temp\best.pt            (Temp model)
```

## Example 2: Manual .env editing
```env
# Before (using local model):
MODEL_PATH=best.pt

# After (using test model):
MODEL_PATH=../Test-Model/best.pt

# Or absolute path:
MODEL_PATH=C:\HARDARE\MicroLabGarbageCollector\Test-Model\best.pt
```

## Example 3: Testing without hardware
```env
# Enable mock modes for testing
MOCK_ESP32=True      # No ESP32 required
MOCK_CAMERA=True     # No camera required
VERBOSE_OUTPUT=True  # See detailed logs
```

## Example 4: Different YOLO models
```env
# Ultra-fast but less accurate
MODEL_PATH=yolov8n.pt

# Balanced speed and accuracy
MODEL_PATH=yolov8s.pt

# Your custom trained model
MODEL_PATH=models/my_custom_model.pt
```

##  Quick Start Commands

1. **List available models:**
   ```bash
   python config_manager.py
   # Select option 1
   ```

2. **Switch to Test-Model:**
   ```bash
   python config_manager.py
   # Select option 2, then choose Test-Model
   ```

3. **Enable testing mode:**
   ```bash
   python config_manager.py
   # Select option 3, then option 3 (enable both mocks)
   ```

4. **Start dashboard:**
   ```bash
   python web_dashboard.py
   ```

5. **View in browser:**
   Open: http://localhost:5000

The system will automatically detect the configuration changes and use the specified model file!