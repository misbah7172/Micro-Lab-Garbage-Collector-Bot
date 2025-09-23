#include <WiFi.h>
#include <DHT.h>
#include <ESP32Servo.h>

// Pin Definitions
// Ultrasonic Sensors (HC-SR04)
#define FRONT_TRIG_PIN    2
#define FRONT_ECHO_PIN    4
#define LEFT_TRIG_PIN     5
#define LEFT_ECHO_PIN     18
#define RIGHT_TRIG_PIN    19
#define RIGHT_ECHO_PIN    21
#define REAR_TRIG_PIN     22
#define REAR_ECHO_PIN     23

// Motor Driver (L298N) Pins
#define MOTOR_LEFT_PWM    25
#define MOTOR_LEFT_DIR1   26
#define MOTOR_LEFT_DIR2   27
#define MOTOR_RIGHT_PWM   32
#define MOTOR_RIGHT_DIR1  33
#define MOTOR_RIGHT_DIR2  14

// Arm Servo Motors
#define ARM_BASE_PIN      12
#define ARM_SHOULDER_PIN  13
#define ARM_ELBOW_PIN     15
#define ARM_GRIPPER_PIN   16

// Bucket Servo Motor
#define BUCKET_SERVO_PIN  17

// Sensors
#define DHT_PIN           35
#define DHT_TYPE          DHT22
#define SMOKE_SENSOR_PIN  34

// Initialize components
DHT dht(DHT_PIN, DHT_TYPE);
Servo armBase, armShoulder, armElbow, armGripper, bucketServo;

// Robot state variables
bool systemActive = true;
unsigned long lastSensorRead = 0;
unsigned long lastMovementCommand = 0;
const unsigned long SENSOR_INTERVAL = 500;  // Read sensors every 500ms
const unsigned long MOVEMENT_TIMEOUT = 2000; // Stop motors after 2 seconds without command

// Motor speeds (0-255)
int motorSpeedNormal = 150;
int motorSpeedSlow = 100;
int motorSpeedFast = 200;

// Servo positions for different arm states
struct ArmPositions {
  int base, shoulder, elbow, gripper;
};

// Predefined arm positions
ArmPositions armHome =      {90, 45, 45, 90};   // Home position
ArmPositions armApproach =  {90, 60, 60, 90};   // Approaching object
ArmPositions armGrabOpen =  {90, 75, 75, 45};   // Gripper open, ready to grab
ArmPositions armGrabClose = {90, 75, 75, 135};  // Gripper closed, holding object
ArmPositions armLiftUp =    {90, 30, 30, 135};  // Lift object up
ArmPositions armClassify =  {45, 60, 60, 135};  // Classification position

// Bucket positions
int bucketMetal = 45;     // Metal objects bucket
int bucketNonMetal = 135; // Non-metal objects bucket
int bucketNeutral = 90;   // Neutral position

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  setupUltrasonicSensors();
  setupMotors();
  setupServos();
  setupSensors();
  
  // Move to home position
  moveArmToPosition(armHome);
  bucketServo.write(bucketNeutral);
  
  Serial.println("ESP32 Garbage Collector Controller Ready");
  Serial.println("Commands: F(orward), B(ackward), L(eft), R(ight), RL(otate Left), RR(otate Right), S(top)");
  Serial.println("Arm: A0(Home), A1(Approach), A2(Grab Open), A3(Grab Close), A4(Lift), A5(Classify)");
  Serial.println("Bucket: B0(Neutral), B1(Metal), B2(Non-Metal)");
}

void setupUltrasonicSensors() {
  pinMode(FRONT_TRIG_PIN, OUTPUT);
  pinMode(FRONT_ECHO_PIN, INPUT);
  pinMode(LEFT_TRIG_PIN, OUTPUT);
  pinMode(LEFT_ECHO_PIN, INPUT);
  pinMode(RIGHT_TRIG_PIN, OUTPUT);
  pinMode(RIGHT_ECHO_PIN, INPUT);
  pinMode(REAR_TRIG_PIN, OUTPUT);
  pinMode(REAR_ECHO_PIN, INPUT);
}

void setupMotors() {
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Stop motors initially
  stopMotors();
}

void setupServos() {
  armBase.attach(ARM_BASE_PIN);
  armShoulder.attach(ARM_SHOULDER_PIN);
  armElbow.attach(ARM_ELBOW_PIN);
  armGripper.attach(ARM_GRIPPER_PIN);
  bucketServo.attach(BUCKET_SERVO_PIN);
}

void setupSensors() {
  dht.begin();
  pinMode(SMOKE_SENSOR_PIN, INPUT);
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read and send sensor data periodically
  if (currentTime - lastSensorRead >= SENSOR_INTERVAL) {
    readAndSendSensorData();
    lastSensorRead = currentTime;
  }
  
  // Auto-stop motors if no command received for timeout period
  if (currentTime - lastMovementCommand >= MOVEMENT_TIMEOUT) {
    stopMotors();
  }
  
  // Process serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  delay(50); // Small delay for stability
}

float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  
  if (duration == 0) {
    return 999.0; // No echo received, assume max distance
  }
  
  float distance = (duration * 0.034) / 2; // Convert to cm
  return constrain(distance, 2.0, 400.0);  // HC-SR04 range
}

void readAndSendSensorData() {
  // Read ultrasonic sensors
  float frontDist = readUltrasonicDistance(FRONT_TRIG_PIN, FRONT_ECHO_PIN);
  float leftDist = readUltrasonicDistance(LEFT_TRIG_PIN, LEFT_ECHO_PIN);
  float rightDist = readUltrasonicDistance(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN);
  float rearDist = readUltrasonicDistance(REAR_TRIG_PIN, REAR_ECHO_PIN);
  
  // Read environmental sensors
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  int smokeLevel = analogRead(SMOKE_SENSOR_PIN);
  
  // Handle sensor errors
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 25.0; // Default value
  }
  if (isnan(humidity)) {
    humidity = 60.0; // Default value
  }
  
  // Send sensor data in formatted string
  Serial.print("SENSORS:");
  Serial.print("F"); Serial.print(frontDist, 1); Serial.print(",");
  Serial.print("L"); Serial.print(leftDist, 1); Serial.print(",");
  Serial.print("R"); Serial.print(rightDist, 1); Serial.print(",");
  Serial.print("B"); Serial.print(rearDist, 1); Serial.print(",");
  Serial.print("T"); Serial.print(temperature, 1); Serial.print(",");
  Serial.print("H"); Serial.print(humidity, 1); Serial.print(",");
  Serial.print("S"); Serial.println(smokeLevel);
}

void processCommand(String command) {
  lastMovementCommand = millis();
  
  // Movement commands
  if (command == "F") {
    moveForward();
    Serial.println("Moving forward");
  }
  else if (command == "B") {
    moveBackward();
    Serial.println("Moving backward");
  }
  else if (command == "L") {
    moveLeft();
    Serial.println("Moving left");
  }
  else if (command == "R") {
    moveRight();
    Serial.println("Moving right");
  }
  else if (command == "RL") {
    rotateLeft();
    Serial.println("Rotating left");
  }
  else if (command == "RR") {
    rotateRight();
    Serial.println("Rotating right");
  }
  else if (command == "S") {
    stopMotors();
    Serial.println("Stopped");
  }
  
  // Arm control commands
  else if (command == "A0") {
    moveArmToPosition(armHome);
    Serial.println("Arm: Home position");
  }
  else if (command == "A1") {
    moveArmToPosition(armApproach);
    Serial.println("Arm: Approach position");
  }
  else if (command == "A2") {
    moveArmToPosition(armGrabOpen);
    Serial.println("Arm: Gripper open");
  }
  else if (command == "A3") {
    moveArmToPosition(armGrabClose);
    Serial.println("Arm: Gripper closed - Collecting");
  }
  else if (command == "A4") {
    moveArmToPosition(armLiftUp);
    Serial.println("Arm: Lifting object");
  }
  else if (command == "A5") {
    moveArmToPosition(armClassify);
    Serial.println("Arm: Classification position");
  }
  
  // Bucket control commands
  else if (command == "B0") {
    bucketServo.write(bucketNeutral);
    Serial.println("Bucket: Neutral position");
  }
  else if (command == "B1") {
    bucketServo.write(bucketMetal);
    Serial.println("Bucket: Metal objects");
  }
  else if (command == "B2") {
    bucketServo.write(bucketNonMetal);
    Serial.println("Bucket: Non-metal objects");
  }
  
  else {
    Serial.println("Unknown command: " + command);
  }
}

// Motor control functions
void moveForward() {
  setMotorDirection(true, true);   // Both motors forward
  setMotorSpeed(motorSpeedNormal, motorSpeedNormal);
}

void moveBackward() {
  setMotorDirection(false, false); // Both motors backward
  setMotorSpeed(motorSpeedNormal, motorSpeedNormal);
}

void moveLeft() {
  setMotorDirection(false, true);  // Left motor backward, right forward
  setMotorSpeed(motorSpeedSlow, motorSpeedSlow);
}

void moveRight() {
  setMotorDirection(true, false);  // Left motor forward, right backward
  setMotorSpeed(motorSpeedSlow, motorSpeedSlow);
}

void rotateLeft() {
  setMotorDirection(false, true);  // Left motor backward, right forward
  setMotorSpeed(motorSpeedNormal, motorSpeedNormal);
}

void rotateRight() {
  setMotorDirection(true, false);  // Left motor forward, right backward
  setMotorSpeed(motorSpeedNormal, motorSpeedNormal);
}

void stopMotors() {
  setMotorSpeed(0, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
}

void setMotorDirection(bool leftForward, bool rightForward) {
  // Left motor direction
  if (leftForward) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
  }
  
  // Right motor direction
  if (rightForward) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  analogWrite(MOTOR_LEFT_PWM, constrain(leftSpeed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain(rightSpeed, 0, 255));
}

void moveArmToPosition(ArmPositions position) {
  armBase.write(position.base);
  delay(200);
  armShoulder.write(position.shoulder);
  delay(200);
  armElbow.write(position.elbow);
  delay(200);
  armGripper.write(position.gripper);
  delay(200);
}