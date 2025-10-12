# Updated Pin Assignments for Environmental Sensors

## ✅ Final Pin Configuration

### MQ2 Smoke Sensor:
- **Analog Pin**: 34 (D34) - Smoke level reading (0-4095)
- **Digital Pin**: 5 (D5) - Smoke detection threshold

### DHT11 Temperature/Humidity:
- **Data Pin**: 35 - Temperature and humidity readings

### Metal Detector (Push Switch):
- **Input Pin**: 17 - Metal detection simulation

### Servo Ramp (Updated):
- **Control Pin**: 16 - Changed from pin 5 to avoid MQ2 conflict

## 🔧 Hardware Connections

### MQ2 Smoke Sensor Wiring:
```
MQ2 Sensor    →    ESP32
VCC           →    3.3V
GND           →    GND
AO (Analog)   →    Pin 34
DO (Digital)  →    Pin 5
```

### DHT11 Temperature/Humidity Wiring:
```
DHT11         →    ESP32
VCC           →    3.3V
GND           →    GND
DATA          →    Pin 35 (with 10kΩ pullup resistor)
```

### Metal Detector (Push Switch) Wiring:
```
Push Switch   →    ESP32
One terminal  →    Pin 17
Other terminal→    GND
(Internal pullup resistor enabled in code)
```

## 📊 Pin Usage Summary

| Component | Pin | Type | Notes |
|-----------|-----|------|-------|
| MQ2 Analog | 34 | ADC | Smoke level (0-4095) |
| MQ2 Digital | 5 | GPIO | Smoke threshold trigger |
| DHT11 Data | 35 | GPIO | Temp/humidity (needs pullup) |
| Metal Detector | 17 | GPIO | Push switch (internal pullup) |
| Servo Ramp | 16 | PWM | Moved from pin 5 |

## 🎯 Changes Made:
1. **MQ2 Analog**: Changed from pin 36 to pin 34 (D34)
2. **MQ2 Digital**: Changed from pin 39 to pin 5 (D5)
3. **Servo Ramp**: Moved from pin 5 to pin 16 to avoid conflict
4. **DHT11**: Remains on pin 35 (no change needed)

All pins are now compatible with standard ESP32 boards and avoid conflicts with existing motor, ultrasonic, and servo assignments.