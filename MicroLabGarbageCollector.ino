#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <ESP32Servo.h>

// Blynk credentials
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_DEVICE_NAME "GarbageCollector"
#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

// WiFi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// Motor driver pins (L298N)
#define MOTOR_LEFT_FORWARD 2
#define MOTOR_LEFT_BACKWARD 4
#define MOTOR_RIGHT_FORWARD 16
#define MOTOR_RIGHT_BACKWARD 17
#define MOTOR_LEFT_ENABLE 5
#define MOTOR_RIGHT_ENABLE 18

// Ultrasonic sensor pins
#define TRIG_PIN 12
#define ECHO_PIN 14

// Servo motor pins for robotic arm
#define SERVO_BASE 25
#define SERVO_ARM 26
#define SERVO_GRIPPER 27

// Environmental sensors
#define DHT_PIN 23
#define DHT_TYPE DHT22
#define SMOKE_SENSOR_PIN 35

// System status LED
#define STATUS_LED 13

// Initialize components
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoBase;
Servo servoArm;
Servo servoGripper;

// System variables
bool systemActive = true;
unsigned long lastSensorRead = 0;
unsigned long lastBlynkUpdate = 0;
unsigned long lastHeartbeat = 0;
const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long BLYNK_INTERVAL = 5000;
const unsigned long HEARTBEAT_TIMEOUT = 10000;

// Movement and arm positions
int basePosition = 90;
int armPosition = 90;
int gripperPosition = 0;
bool isCollecting = false;

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_ENABLE, OUTPUT);
  pinMode(MOTOR_RIGHT_ENABLE, OUTPUT);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize servos
  servoBase.attach(SERVO_BASE);
  servoArm.attach(SERVO_ARM);
  servoGripper.attach(SERVO_GRIPPER);
  
  // Set initial servo positions
  resetArmPosition();
  
  // Initialize sensors
  dht.begin();
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
  }
  Serial.println("WiFi connected!");
  digitalWrite(STATUS_LED, HIGH);
  
  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
  
  Serial.println("Micro Lab Garbage Collector Ready!");
  stopMotors();
}

void loop() {
  Blynk.run();
  
  // Check for serial commands from Python script
  if (Serial.available()) {
    char command = Serial.read();
    lastHeartbeat = millis();
    
    if (systemActive) {
      processCommand(command);
    }
  }
  
  // Check for communication timeout (failsafe)
  if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    if (systemActive) {
      Serial.println("Communication lost - Stopping system");
      stopMotors();
      systemActive = false;
      Blynk.virtualWrite(V5, "OFFLINE");
    }
  }
  
  // Read environmental sensors
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    readEnvironmentalSensors();
    lastSensorRead = millis();
  }
  
  // Update Blynk
  if (millis() - lastBlynkUpdate > BLYNK_INTERVAL) {
    updateBlynk();
    lastBlynkUpdate = millis();
  }
  
  delay(50);
}

void processCommand(char command) {
  switch (command) {
    case 'F': // Move forward
      if (getDistance() > 20) { // Safe distance check
        moveForward();
        Serial.println("Moving forward");
      } else {
        stopMotors();
        collectTrash();
        Serial.println("Obstacle detected - Collecting trash");
      }
      break;
      
    case 'R': // Rotate to search
      rotateRight();
      Serial.println("Rotating to search");
      break;
      
    case 'S': // Stop
      stopMotors();
      Serial.println("Stopped");
      break;
      
    case 'C': // Collect trash
      collectTrash();
      break;
      
    case 'H': // Heartbeat
      systemActive = true;
      break;
      
    default:
      Serial.println("Unknown command");
      break;
  }
}

// Motor control functions
void moveForward() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, 200);
  analogWrite(MOTOR_RIGHT_ENABLE, 200);
}

void rotateRight() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, 150);
  analogWrite(MOTOR_RIGHT_ENABLE, 150);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, 0);
  analogWrite(MOTOR_RIGHT_ENABLE, 0);
}

// Ultrasonic sensor function
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  
  return distance;
}

// Robotic arm functions
void collectTrash() {
  if (isCollecting) return;
  
  isCollecting = true;
  Serial.println("Starting trash collection sequence");
  
  // Stop movement
  stopMotors();
  delay(500);
  
  // Lower arm
  moveServoSmoothly(servoArm, armPosition, 45);
  armPosition = 45;
  delay(500);
  
  // Open gripper
  moveServoSmoothly(servoGripper, gripperPosition, 90);
  gripperPosition = 90;
  delay(500);
  
  // Move forward slightly
  moveForward();
  delay(1000);
  stopMotors();
  
  // Close gripper
  moveServoSmoothly(servoGripper, gripperPosition, 0);
  gripperPosition = 0;
  delay(1000);
  
  // Lift arm
  moveServoSmoothly(servoArm, armPosition, 90);
  armPosition = 90;
  delay(500);
  
  // Move to disposal position
  moveServoSmoothly(servoBase, basePosition, 180);
  basePosition = 180;
  delay(1000);
  
  // Lower arm to drop
  moveServoSmoothly(servoArm, armPosition, 45);
  armPosition = 45;
  delay(500);
  
  // Open gripper to drop trash
  moveServoSmoothly(servoGripper, gripperPosition, 90);
  gripperPosition = 90;
  delay(1000);
  
  // Reset arm position
  resetArmPosition();
  
  Serial.println("Trash collection completed");
  isCollecting = false;
}

void resetArmPosition() {
  moveServoSmoothly(servoBase, basePosition, 90);
  basePosition = 90;
  delay(500);
  
  moveServoSmoothly(servoArm, armPosition, 90);
  armPosition = 90;
  delay(500);
  
  moveServoSmoothly(servoGripper, gripperPosition, 0);
  gripperPosition = 0;
  delay(500);
}

void moveServoSmoothly(Servo &servo, int currentPos, int targetPos) {
  int step = (targetPos > currentPos) ? 1 : -1;
  
  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(15);
  }
  servo.write(targetPos);
}

// Environmental sensor functions
void readEnvironmentalSensors() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int smokeLevel = analogRead(SMOKE_SENSOR_PIN);
  
  // Check for sensor errors
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("DHT sensor error");
    return;
  }
  
  Serial.printf("Temp: %.1f°C, Humidity: %.1f%%, Smoke: %d\n", 
                temperature, humidity, smokeLevel);
  
  // Check for dangerous conditions
  if (temperature > 50 || smokeLevel > 500) {
    Serial.println("WARNING: High temperature or smoke detected!");
    stopMotors();
    Blynk.logEvent("danger_alert", "High temperature or smoke detected!");
  }
}

// Blynk functions
void updateBlynk() {
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int smokeLevel = analogRead(SMOKE_SENSOR_PIN);
  long distance = getDistance();
  
  if (!isnan(temperature) && !isnan(humidity)) {
    Blynk.virtualWrite(V1, temperature);
    Blynk.virtualWrite(V2, humidity);
  }
  
  Blynk.virtualWrite(V3, smokeLevel);
  Blynk.virtualWrite(V4, distance);
  Blynk.virtualWrite(V5, systemActive ? "ONLINE" : "OFFLINE");
  Blynk.virtualWrite(V6, isCollecting ? "COLLECTING" : "SEARCHING");
}

// Blynk virtual pin handlers
BLYNK_WRITE(V0) {
  int value = param.asInt();
  if (value == 1) {
    systemActive = true;
    Serial.println("System activated via Blynk");
  } else {
    systemActive = false;
    stopMotors();
    Serial.println("System deactivated via Blynk");
  }
}

BLYNK_WRITE(V7) {
  int value = param.asInt();
  if (value == 1 && systemActive) {
    resetArmPosition();
    Serial.println("Arm reset via Blynk");
  }
}
