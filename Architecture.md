##  System Architecture

### Overview
```
┌─────────────────┐    USB/Serial    ┌─────────────────┐
│     LAPTOP      │◄────────────────►│      ESP32      │
│                 │                  │   (On Robot)    │
│ • Webcam        │                  │                 │
│ • AI Detection  │                  │ • Motor Control │
│ • Python Script │                  │ • Sensors       │
│ • Live Video    │                  │ • Robotic Arm   │
└─────────────────┘                  │ • Blynk IoT     │
                                     └─────────────────┘
```

### Complete Wiring Schematic

```
                    ┌─────────────┐
                    │    ESP32    │
                    │             │
      ┌─────────────┤ GPIO 2      │
      │         ┌───┤ GPIO 4      │
      │         │┌──┤ GPIO 5      │
      │         ││┌─┤ GPIO 12     │
      │         │││ │ GPIO 14     │──── HC-SR04 ECHO
      │         │││ │ GPIO 16     │──── L298N IN3
      │         │││ │ GPIO 17     │──── L298N IN4
      │         │││ │ GPIO 18     │──── L298N ENB
      │         │││ │ GPIO 23     │──── DHT22 Data
      │         │││ │ GPIO 25     │──── Base Servo
      │         │││ │ GPIO 26     │──── Arm Servo
      │         │││ │ GPIO 27     │──── Gripper Servo
      │         │││ │ GPIO 35     │──── MQ-2 Analog
      │         │││ │ VIN         │──── Battery +
      │         │││ │ GND         │──── Common Ground
      │         │││ └─────────────┘
      │         │││
      │         │││  ┌─────────────┐
      │         │││  │    L298N    │
      │         ││└──┤ IN1         │
      │         │└───┤ IN2         │
      │         └────┤ ENA         │
      │              │ IN3         │
      │              │ IN4         │
      │              │ ENB         │
      │              │ OUT1        │──── Left Motor +
      │              │ OUT2        │──── Left Motor -
      │              │ OUT3        │──── Right Motor +
      │              │ OUT4        │──── Right Motor -
      │              └─────────────┘
      │
      └────── L298N IN1
      
           ┌─────────────┐
           │   DHT22     │
           │ VCC GND Data│
           └──┬───┬───┬──┘
              │   │   └── GPIO 23
              │   └────── GND
              └────────── 3.3V
```
