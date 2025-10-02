#include <DHT.h>
#include <ESP32Servo.h>

// Motor driver pins (L298N)
#define MOTOR_LEFT_FORWARD 2
#define MOTOR_LEFT_BACKWARD 4
#define MOTOR_RIGHT_FORWARD 16
#define MOTOR_RIGHT_BACKWARD 17
#define MOTOR_LEFT_ENABLE 5
#define MOTOR_RIGHT_ENABLE 18

// Ultrasonic sensor pins (3 sensors)
#define TRIG_LEFT 12
#define ECHO_LEFT 14
#define TRIG_MIDDLE 19
#define ECHO_MIDDLE 21
#define TRIG_RIGHT 22
#define ECHO_RIGHT 32

// Servo motor pins
#define SERVO_ARM_LEFT 25
#define SERVO_ARM_RIGHT 26
#define SERVO_BOX_LEFT 27
#define SERVO_BOX_RIGHT 33
#define SERVO_RAMP 15

// Control switches
#define EMERGENCY_STOP_SWITCH 34
#define PROCESS_CONTROL_SWITCH 35

// Environmental sensors
#define DHT_PIN 23
#define DHT_TYPE DHT22
#define SMOKE_SENSOR_PIN 36

// System status LED
#define STATUS_LED 13

// Motor speed constants
#define FULL_SPEED 255
#define SLOW_SPEED 120
#define STOP_SPEED 0

// Initialize components
DHT dht(DHT_PIN, DHT_TYPE);
Servo servoArmLeft;
Servo servoArmRight;
Servo servoBoxLeft;
Servo servoBoxRight;
Servo servoRamp;

// Robot states
enum RobotState {
  STOPPED,
  SEARCHING,
  TRACKING,
  ULTRASONIC_RANGE,
  COLLECTING
};

// System variables
RobotState currentState = STOPPED;
bool emergencyStop = false;
bool processRunning = false;
bool systemActive = true;
unsigned long lastSensorRead = 0;
unsigned long lastHeartbeat = 0;
unsigned long searchStartTime = 0;
unsigned long forwardStartTime = 0;
int rotationCount = 0;
bool movingForward = false;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long HEARTBEAT_TIMEOUT = 10000;
const unsigned long ROTATION_TIME = 3000; // Time for one rotation
const unsigned long FORWARD_TIME = 2000;  // Forward movement time

// Servo positions
int armLeftPosition = 60;
int armRightPosition = 60;
int boxLeftPosition = 0;   // 0 = open
int boxRightPosition = 0;  // 0 = open
int rampPosition = 0;      // 0 = closed

// Camera and object tracking
bool objectDetected = false;
bool objectCentered = false;
float objectDistance = 0.0;
int objectCenterX = 0;
int frameCenterX = 320; // Assuming 640x480 camera
int centerTolerance = 50;

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
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MIDDLE, OUTPUT);
  pinMode(ECHO_MIDDLE, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize control switches
  pinMode(EMERGENCY_STOP_SWITCH, INPUT_PULLUP);
  pinMode(PROCESS_CONTROL_SWITCH, INPUT_PULLUP);
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize servos
  servoArmLeft.attach(SERVO_ARM_LEFT);
  servoArmRight.attach(SERVO_ARM_RIGHT);
  servoBoxLeft.attach(SERVO_BOX_LEFT);
  servoBoxRight.attach(SERVO_BOX_RIGHT);
  servoRamp.attach(SERVO_RAMP);
  
  // Set initial servo positions
  resetToInitialPosition();
  
  // Initialize sensors
  dht.begin();
  
  Serial.println("Advanced Garbage Collector Ready!");
  Serial.println("States: STOPPED->SEARCHING->TRACKING->ULTRASONIC_RANGE->COLLECTING");
  stopMotors();
}

void loop() {
  // Check control switches
  checkControlSwitches();
  
  // Only proceed if not in emergency stop and process is running
  if (!emergencyStop && processRunning) {
    // Check for serial commands from Python script
    if (Serial.available()) {
      processSerialCommand();
      lastHeartbeat = millis();
    }
    
    // Main state machine
    executeStateMachine();
  } else {
    // Emergency stop or process paused
    stopMotors();
    currentState = STOPPED;
  }
  
  // Check for communication timeout (failsafe)
  if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    if (systemActive) {
      Serial.println("Communication lost - Stopping system");
      stopMotors();
      systemActive = false;
      currentState = STOPPED;
    }
  }
  
  // Read environmental sensors
  if (millis() - lastSensorRead > SENSOR_INTERVAL) {
    readEnvironmentalSensors();
    lastSensorRead = millis();
  }
  
  delay(50);
}

// Control switch monitoring
void checkControlSwitches() {
  emergencyStop = !digitalRead(EMERGENCY_STOP_SWITCH);
  processRunning = !digitalRead(PROCESS_CONTROL_SWITCH);
  
  // LED indication
  if (emergencyStop) {
    digitalWrite(STATUS_LED, HIGH); // Solid on for emergency
  } else if (!processRunning) {
    // Blink for paused
    digitalWrite(STATUS_LED, (millis() / 500) % 2);
  } else {
    digitalWrite(STATUS_LED, LOW); // Off for normal operation
  }
}

// Serial command processing
void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.startsWith("OBJECT_DETECTED:")) {
    // Format: OBJECT_DETECTED:x_position,distance
    int commaIndex = command.indexOf(',');
    if (commaIndex > 0) {
      objectCenterX = command.substring(16, commaIndex).toInt();
      objectDistance = command.substring(commaIndex + 1).toFloat();
      objectDetected = true;
      
      // Check if object is centered
      objectCentered = abs(objectCenterX - frameCenterX) <= centerTolerance;
      
      if (currentState == SEARCHING) {
        currentState = TRACKING;
        Serial.println("STATE: TRACKING");
      }
    }
  }
  else if (command == "NO_OBJECT") {
    objectDetected = false;
    objectCentered = false;
    if (currentState == TRACKING || currentState == ULTRASONIC_RANGE) {
      currentState = SEARCHING;
      Serial.println("STATE: SEARCHING");
    }
  }
  else if (command == "START_SEARCH") {
    if (currentState == STOPPED) {
      currentState = SEARCHING;
      searchStartTime = millis();
      rotationCount = 0;
      movingForward = false;
      Serial.println("STATE: SEARCHING");
    }
  }
  else if (command == "HEARTBEAT") {
    systemActive = true;
    lastHeartbeat = millis();
  }
}

// Main state machine
void executeStateMachine() {
  switch (currentState) {
    case STOPPED:
      stopMotors();
      break;
      
    case SEARCHING:
      executeSearchPattern();
      break;
      
    case TRACKING:
      executeObjectTracking();
      break;
      
    case ULTRASONIC_RANGE:
      executeUltrasonicDetection();
      break;
      
    case COLLECTING:
      executeCollectionSequence();
      break;
  }
}

// Search pattern execution
void executeSearchPattern() {
  if (!movingForward) {
    // Rotation phase
    rotateClockwise(FULL_SPEED);
    
    // Check if completed 2 rotations
    if (millis() - searchStartTime >= ROTATION_TIME * 2) {
      stopMotors();
      movingForward = true;
      forwardStartTime = millis();
      rotationCount++;
      Serial.println("SEARCH: Moving forward");
    }
  } else {
    // Forward movement phase
    moveForward(FULL_SPEED);
    
    // Check if completed forward movement
    if (millis() - forwardStartTime >= FORWARD_TIME) {
      stopMotors();
      movingForward = false;
      searchStartTime = millis();
      Serial.println("SEARCH: Resuming rotation");
    }
  }
}

// Object tracking execution
void executeObjectTracking() {
  if (!objectDetected) {
    stopMotors();
    currentState = SEARCHING;
    return;
  }
  
  if (!objectCentered) {
    // Stop and center the object
    stopMotors();
    delay(100); // Brief pause
    
    if (objectCenterX < frameCenterX - centerTolerance) {
      // Object is left, turn left
      turnLeft(SLOW_SPEED);
      delay(200);
      stopMotors();
    } else if (objectCenterX > frameCenterX + centerTolerance) {
      // Object is right, turn right
      turnRight(SLOW_SPEED);
      delay(200);
      stopMotors();
    }
  } else {
    // Object is centered, move forward slowly
    if (objectDistance > 0.1) { // More than 10cm
      moveForward(SLOW_SPEED);
    } else {
      // Within 10cm, switch to ultrasonic detection
      stopMotors();
      currentState = ULTRASONIC_RANGE;
      Serial.println("STATE: ULTRASONIC_RANGE");
    }
  }
}

// Ultrasonic detection execution
void executeUltrasonicDetection() {
  long leftDistance = getUltrasonicDistance(TRIG_LEFT, ECHO_LEFT);
  long middleDistance = getUltrasonicDistance(TRIG_MIDDLE, ECHO_MIDDLE);
  long rightDistance = getUltrasonicDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  Serial.printf("Ultrasonic - L:%ld M:%ld R:%ld\n", leftDistance, middleDistance, rightDistance);
  
  if (middleDistance < 15) { // Object detected in middle sensor
    stopMotors();
    currentState = COLLECTING;
    Serial.println("STATE: COLLECTING");
  } else if (leftDistance < 20) { // Object detected on left
    moveCircularLeft(FULL_SPEED);
  } else if (rightDistance < 20) { // Object detected on right
    moveCircularRight(FULL_SPEED);
  } else {
    // No object in ultrasonic range, return to tracking
    currentState = TRACKING;
  }
}

// Motor control functions
void moveForward(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
}

void rotateClockwise(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
}

void moveCircularLeft(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, speed / 2);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
}

void moveCircularRight(int speed) {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed / 2);
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
long getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;
  
  return distance;
}

// Collection sequence execution
void executeCollectionSequence() {
  static int collectionStep = 0;
  static unsigned long stepStartTime = 0;
  
  if (collectionStep == 0) {
    Serial.println("COLLECTION: Starting sequence");
    stopMotors();
    stepStartTime = millis();
    collectionStep = 1;
  }
  
  switch (collectionStep) {
    case 1: // Lower arms (60° to 20°)
      if (millis() - stepStartTime >= 500) {
        moveServosSmoothly(servoArmLeft, servoArmRight, armLeftPosition, armRightPosition, 20, 20);
        armLeftPosition = 20;
        armRightPosition = 20;
        Serial.println("COLLECTION: Arms lowered to 20°");
        stepStartTime = millis();
        collectionStep = 2;
      }
      break;
      
    case 2: // Wait for object to enter box
      if (millis() - stepStartTime >= 2000) {
        Serial.println("COLLECTION: Object in box, closing box");
        stepStartTime = millis();
        collectionStep = 3;
      }
      break;
      
    case 3: // Close box (0° to 45°)
      moveServosSmoothly(servoBoxLeft, servoBoxRight, boxLeftPosition, boxRightPosition, 45, 45);
      boxLeftPosition = 45;
      boxRightPosition = 45;
      Serial.println("COLLECTION: Box closed");
      stepStartTime = millis();
      collectionStep = 4;
      break;
      
    case 4: // Lift arms (20° to 150°)
      if (millis() - stepStartTime >= 1000) {
        moveServosSmoothly(servoArmLeft, servoArmRight, armLeftPosition, armRightPosition, 150, 150);
        armLeftPosition = 150;
        armRightPosition = 150;
        Serial.println("COLLECTION: Arms lifted to 150°");
        stepStartTime = millis();
        collectionStep = 5;
      }
      break;
      
    case 5: // Open box to release (45° to 0°)
      if (millis() - stepStartTime >= 1000) {
        moveServosSmoothly(servoBoxLeft, servoBoxRight, boxLeftPosition, boxRightPosition, 0, 0);
        boxLeftPosition = 0;
        boxRightPosition = 0;
        Serial.println("COLLECTION: Box opened, releasing object");
        stepStartTime = millis();
        collectionStep = 6;
      }
      break;
      
    case 6: // Open ramp (0° to 90°)
      if (millis() - stepStartTime >= 500) {
        moveServoSmoothly(servoRamp, rampPosition, 90);
        rampPosition = 90;
        Serial.println("COLLECTION: Ramp opened");
        stepStartTime = millis();
        collectionStep = 7;
      }
      break;
      
    case 7: // Close ramp (90° to 0°)
      if (millis() - stepStartTime >= 2000) {
        moveServoSmoothly(servoRamp, rampPosition, 0);
        rampPosition = 0;
        Serial.println("COLLECTION: Ramp closed");
        stepStartTime = millis();
        collectionStep = 8;
      }
      break;
      
    case 8: // Reset arms (150° to 60°)
      if (millis() - stepStartTime >= 500) {
        moveServosSmoothly(servoArmLeft, servoArmRight, armLeftPosition, armRightPosition, 60, 60);
        armLeftPosition = 60;
        armRightPosition = 60;
        Serial.println("COLLECTION: Arms reset to 60°");
        Serial.println("COLLECTION: Sequence completed");
        collectionStep = 0;
        currentState = SEARCHING;
        searchStartTime = millis();
        objectDetected = false;
        objectCentered = false;
      }
      break;
  }
}

// Servo control functions
void resetToInitialPosition() {
  // Set arms to 60 degrees
  servoArmLeft.write(60);
  servoArmRight.write(60);
  armLeftPosition = 60;
  armRightPosition = 60;
  
  // Set box servos to 0 degrees (open)
  servoBoxLeft.write(0);
  servoBoxRight.write(0);
  boxLeftPosition = 0;
  boxRightPosition = 0;
  
  // Set ramp to 0 degrees (closed)
  servoRamp.write(0);
  rampPosition = 0;
  
  delay(1000); // Allow servos to reach position
  Serial.println("Servos reset to initial positions");
}

void moveServoSmoothly(Servo &servo, int currentPos, int targetPos) {
  int step = (targetPos > currentPos) ? 1 : -1;
  
  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(15);
  }
  servo.write(targetPos);
}

void moveServosSmoothly(Servo &servo1, Servo &servo2, int current1, int current2, int target1, int target2) {
  int maxSteps = max(abs(target1 - current1), abs(target2 - current2));
  
  for (int i = 0; i <= maxSteps; i++) {
    int pos1 = current1 + (target1 - current1) * i / maxSteps;
    int pos2 = current2 + (target2 - current2) * i / maxSteps;
    
    servo1.write(pos1);
    servo2.write(pos2);
    delay(15);
  }
  
  servo1.write(target1);
  servo2.write(target2);
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
  
  Serial.printf("ENV - Temp: %.1f°C, Humidity: %.1f%%, Smoke: %d\n", 
                temperature, humidity, smokeLevel);
  
  // Check for dangerous conditions
  if (temperature > 50 || smokeLevel > 500) {
    Serial.println("WARNING: High temperature or smoke detected!");
    emergencyStop = true;
    stopMotors();
  }
}