#include <ESP32Servo.h>

// Configuration options
#define GENTLE_SERVO_STARTUP true  // Set to false to disable auto servo movement on startup

// Servo type configuration
#define ARM_SERVO_TYPE_SG92R true   // Arms use SG92R servos (more stable)
#define RAMP_SERVO_TYPE_SG90 true   // Ramp uses SG90 servo (lighter duty)

// Motor driver pins (L298N) - ESP32 GUARANTEED SAFE PINS
// Motor A (Left): ENA=18, IN1=22, IN2=23  
// Motor B (Right): ENB=33, IN3=32, IN4=14
#define MOTOR_LEFT_FORWARD 22    // IN1 - Safe GPIO pin
#define MOTOR_LEFT_BACKWARD 23   // IN2 - Safe GPIO pin  
#define MOTOR_RIGHT_FORWARD 32   // IN3 - Safe GPIO pin 
#define MOTOR_RIGHT_BACKWARD 14  // IN4 - Safe GPIO pin
#define MOTOR_LEFT_ENABLE 18     // ENA - PWM capable pin
#define MOTOR_RIGHT_ENABLE 33    // ENB - PWM capable pin

// Ultrasonic sensor pins (3 sensors) - UPDATED FOR NO CONFLICTS
#define TRIG_LEFT 12
#define ECHO_LEFT 15             // Changed from 14 to avoid motor conflict
#define TRIG_MIDDLE 13           
#define ECHO_MIDDLE 27           
#define TRIG_RIGHT 26            // Changed from 32 to avoid motor conflict  
#define ECHO_RIGHT 25            // Changed from 33 to avoid motor conflict

// Servo motor pins - UPDATED FOR NO CONFLICTS
#define SERVO_ARM_LEFT 4         // Changed from 25 to avoid ultrasonic conflict
#define SERVO_ARM_RIGHT 2        // Changed from 26 to avoid ultrasonic conflict  
#define SERVO_BOX_LEFT 21        
#define SERVO_BOX_RIGHT 19       
#define SERVO_RAMP 5             // Changed from 15 to avoid ultrasonic conflict

// System status LED
#define STATUS_LED 13

// Motor speed constants
#define FULL_SPEED 255
#define SLOW_SPEED 160
#define STOP_SPEED 0

// Initialize components
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
bool systemActive = true;
unsigned long lastSensorRead = 0;
unsigned long lastHeartbeat = 0;
unsigned long searchStartTime = 0;
unsigned long forwardStartTime = 0;
int rotationCount = 0;
bool movingForward = false;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long HEARTBEAT_TIMEOUT = 10000;
const unsigned long ROTATION_TIME = 8000; // Time for one rotation (increased for longer movement)
const unsigned long FORWARD_TIME = 4000;  // Forward movement time (increased for longer movement)

// Servo positions
int armLeftPosition = 60;   // Left arm servo position (facing front, 60° up)
int armRightPosition = 90;  // Right arm servo position (facing back, 90° for synchronized parallel up position)
int boxLeftPosition = 0;    // 0 = open
int boxRightPosition = 0;   // 0 = open
int rampPosition = 0;       // 0 = closed

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
  
  // Initialize status LED
  pinMode(STATUS_LED, OUTPUT);
  
  // Initialize servos with servo-type specific settings
  // SG92R servos (Arms) - More stable, need precise timing
  servoArmLeft.attach(SERVO_ARM_LEFT, 500, 2500);  // SG92R pulse width range
  servoArmRight.attach(SERVO_ARM_RIGHT, 500, 2500);
  
  // Box servos (assuming SG92R for consistency)
  servoBoxLeft.attach(SERVO_BOX_LEFT, 500, 2500);
  servoBoxRight.attach(SERVO_BOX_RIGHT, 500, 2500);
  
  // SG90 servo (Ramp) - Different pulse width range
  servoRamp.attach(SERVO_RAMP, 544, 2400);  // SG90 pulse width range
  
  // Wait for servo attachment to stabilize
  delay(1000);  // Longer delay for mixed servo types
  
  // Set initial servo positions based on configuration
  #if GENTLE_SERVO_STARTUP
    resetToInitialPositionGently();
  #else
    // Just set position variables without moving servos (synchronized positions)
    armLeftPosition = 60;   // Left arm (front-facing, up)
    armRightPosition = 90;  // Right arm (back-facing, synchronized up)
    boxLeftPosition = 0;
    boxRightPosition = 0;
    rampPosition = 0;
    Serial.println("Servos attached - positions not auto-set (gentle startup disabled)");
  #endif
  
  Serial.println("Advanced Garbage Collector Ready!");
  Serial.println("States: STOPPED->SEARCHING->TRACKING->ULTRASONIC_RANGE->COLLECTING");
  
  // Print motor pin configuration for debugging
  Serial.println("=== Motor Pin Configuration ===");
  Serial.println("Left Motor (A): ENA=" + String(MOTOR_LEFT_ENABLE) + 
                ", IN1=" + String(MOTOR_LEFT_FORWARD) + 
                ", IN2=" + String(MOTOR_LEFT_BACKWARD));
  Serial.println("Right Motor (B): ENB=" + String(MOTOR_RIGHT_ENABLE) + 
                ", IN3=" + String(MOTOR_RIGHT_FORWARD) + 
                ", IN4=" + String(MOTOR_RIGHT_BACKWARD));
    Serial.println("===============================");
  
  // Run motor test on startup for debugging
  delay(2000);  // Wait 2 seconds before testing
  
  // Test pin validity first
  testPinValidity();
  testMotors();
  
  // Wait for Python command to start (don't auto-start)
  Serial.println("🔧 SYSTEM READY - Waiting for Python commands...");
  Serial.println("� Send 'START_SEARCH' command from Python to begin operation");
  currentState = STOPPED;  // Start in STOPPED state
  Serial.println("STATE: STOPPED");
}

// Test if pins are valid on this ESP32
void testPinValidity() {
  Serial.println("=== ESP32 PIN VALIDITY TEST ===");
  
  int motorPins[] = {MOTOR_LEFT_FORWARD, MOTOR_LEFT_BACKWARD, MOTOR_RIGHT_FORWARD, 
                     MOTOR_RIGHT_BACKWARD, MOTOR_LEFT_ENABLE, MOTOR_RIGHT_ENABLE};
  String pinNames[] = {"LEFT_FORWARD", "LEFT_BACKWARD", "RIGHT_FORWARD", 
                       "RIGHT_BACKWARD", "LEFT_ENABLE", "RIGHT_ENABLE"};
  
  for (int i = 0; i < 6; i++) {
    Serial.print("Testing pin " + String(motorPins[i]) + " (" + pinNames[i] + "): ");
    
    // Test if pin can be set as output
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], HIGH);
    delay(10);
    
    if (digitalRead(motorPins[i]) == HIGH) {
      digitalWrite(motorPins[i], LOW);
      delay(10);
      if (digitalRead(motorPins[i]) == LOW) {
        Serial.println("✅ VALID");
      } else {
        Serial.println("❌ STUCK HIGH");
      }
    } else {
      Serial.println("❌ CANNOT SET HIGH");
    }
  }
  Serial.println("===============================");
}

void loop() {
  // Debug loop execution
  static unsigned long lastLoopDebug = 0;
  if (millis() - lastLoopDebug >= 2000) {
    Serial.println("🔧 LOOP DEBUG: systemActive=" + String(systemActive) + ", currentState=" + String(currentState));
    lastLoopDebug = millis();
  }
  
  // Check for serial commands from Python script FIRST
  if (Serial.available()) {
    processSerialCommand();
    lastHeartbeat = millis();
    // Don't automatically reactivate system - only START_SEARCH should do that
  }
  
  // Proceed if system is active
  if (systemActive) {
    // Main state machine
    Serial.println("🔧 CALLING executeStateMachine()");
    executeStateMachine();
  } else {
    // System inactive
    Serial.println("🔧 STOPPING MOTORS - systemActive=" + String(systemActive));
    stopMotors();
    currentState = STOPPED;
  }
  
  // Check for communication timeout (failsafe) - only if Python has connected before
  if (lastHeartbeat > 0 && millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
    if (systemActive) {
      Serial.println("⚠️ Communication lost - Stopping system for safety");
      stopMotors();
      systemActive = false;
      currentState = STOPPED;
    }
  }
  
  delay(50);
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
    // Temporarily disable automatic state change to SEARCHING
    // if (currentState == TRACKING || currentState == ULTRASONIC_RANGE) {
    //   currentState = SEARCHING;
    //   Serial.println("STATE: SEARCHING");
    // }
    Serial.println("🔍 NO_OBJECT received - staying in current state for debugging");
  }
  else if (command == "START_SEARCH") {
    systemActive = true; // Reactivate system
    currentState = SEARCHING;
    searchStartTime = millis();
    rotationCount = 0;
    movingForward = false;
    Serial.println("STATE: SEARCHING");
    Serial.println("🚀 SEARCH STARTED BY PYTHON COMMAND");
  }
  else if (command == "HEARTBEAT") {
    // Update heartbeat but don't reactivate stopped system
    lastHeartbeat = millis();
  }
  else if (command == "STOP") {
    stopMotors();
    currentState = STOPPED;
    objectDetected = false;
    objectCentered = false;
    systemActive = false; // Stop autonomous operation
    Serial.println("STATE: STOPPED");
    Serial.println("🛑 SYSTEM STOPPED BY PYTHON COMMAND");
  }
  else if (command == "PAUSE") {
    stopMotors();
    currentState = STOPPED;
    objectDetected = false;
    objectCentered = false;
    systemActive = false; // Pause autonomous operation
    Serial.println("STATE: STOPPED");
    Serial.println("⏸️ SYSTEM PAUSED BY PYTHON COMMAND");
  }
  else if (command == "MOTOR_TEST") {
    // Simple motor test for debugging
    Serial.println("=== MOTOR TEST START ===");
    Serial.println("Testing Left Motor Forward...");
    moveForward(150);
    delay(1000);
    
    Serial.println("Testing Right Motor Forward...");
    stopMotors();
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(MOTOR_RIGHT_ENABLE, 150);
    delay(1000);
    
    stopMotors();
    Serial.println("=== MOTOR TEST COMPLETE ===");
  }
}

// Main state machine
void executeStateMachine() {
  // Debug output for state changes
  static RobotState lastState = STOPPED;
  if (currentState != lastState) {
    Serial.println("🔄 STATE CHANGE: " + String(lastState) + " -> " + String(currentState));
    lastState = currentState;
  }
  
  // Always print which case we're entering
  Serial.println("🔧 STATE MACHINE: Entering case " + String(currentState));
  
  switch (currentState) {
    case STOPPED:
      Serial.println("🔧 STATE MACHINE: STOPPED case - calling stopMotors()");
      stopMotors();
      break;
      
    case SEARCHING:
      Serial.println("🔧 STATE MACHINE: SEARCHING case - calling executeSearchPattern()");
      executeSearchPattern();
      break;
      
    case TRACKING:
      Serial.println("🔧 STATE MACHINE: TRACKING case");
      executeObjectTracking();
      break;
      
    case ULTRASONIC_RANGE:
      Serial.println("🔧 STATE MACHINE: ULTRASONIC_RANGE case");
      executeUltrasonicDetection();
      break;
      
    case COLLECTING:
      Serial.println("🔧 STATE MACHINE: COLLECTING case");
      executeCollectionSequence();
      break;
  }
  
  Serial.println("🔧 STATE MACHINE: Exiting case " + String(currentState));
}

// Search pattern execution
void executeSearchPattern() {
  // Debug output to confirm search is running
  static unsigned long lastDebugOutput = 0;
  if (millis() - lastDebugOutput >= 3000) {  // Print every 3 seconds
    Serial.println("🔄 SEARCH: Continuous clockwise rotation...");
    Serial.println("🔧 DEBUG: System active=" + String(systemActive));
    Serial.println("🔧 DEBUG: Current state=" + String(currentState));
    lastDebugOutput = millis();
  }
  
  // PURE CONTINUOUS rotation search - no stopping, no alternating, no forward movement
  Serial.println("🔧 DEBUG: Calling rotateClockwise with speed " + String(FULL_SPEED));
  rotateClockwise(FULL_SPEED);
}

// Object tracking execution
void executeObjectTracking() {
  // Debug output
  static unsigned long lastTrackingDebug = 0;
  if (millis() - lastTrackingDebug >= 2000) {
    Serial.println("🎯 TRACKING: Object detected=" + String(objectDetected) + ", centered=" + String(objectCentered));
    lastTrackingDebug = millis();
  }
  
  if (!objectDetected) {
    stopMotors();
    currentState = SEARCHING;
    Serial.println("🔄 TRACKING -> SEARCHING: No object detected");
    return;
  }
  
  if (!objectCentered) {
    // Continuously turn to center the object
    if (objectCenterX < frameCenterX - centerTolerance) {
      // Object is left, keep turning left
      turnLeft(SLOW_SPEED);
    } else if (objectCenterX > frameCenterX + centerTolerance) {
      // Object is right, keep turning right
      turnRight(SLOW_SPEED);
    }
  } else {
    // Object is centered, move forward continuously
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
    moveCircularLeft(FULL_SPEED);  // Keep moving continuously
  } else if (rightDistance < 20) { // Object detected on right
    moveCircularRight(FULL_SPEED); // Keep moving continuously
  } else {
    // No object in ultrasonic range, keep searching
    rotateClockwise(SLOW_SPEED);  // Continuous slow rotation
  }
}
// Motor testing function
void testMotors() {
  Serial.println("=== MOTOR TEST START ===");
  
  // Test Motor A (Left)
  Serial.println("Testing Left Motor Forward...");
  analogWrite(MOTOR_LEFT_ENABLE, 255);  // Use analogWrite for proper enable control
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  delay(1000);
  
  Serial.println("Testing Left Motor Backward...");
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  delay(1000);
  
  // Stop Left Motor
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  analogWrite(MOTOR_LEFT_ENABLE, 0);  // Use analogWrite for proper enable control
  
  // Test Motor B (Right)
  Serial.println("Testing Right Motor Forward...");
  analogWrite(MOTOR_RIGHT_ENABLE, 255);  // Use analogWrite for proper enable control
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  delay(1000);
  
  Serial.println("Testing Right Motor Backward...");
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  delay(1000);
  
  // Stop Right Motor
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
  analogWrite(MOTOR_RIGHT_ENABLE, 0);  // Use analogWrite for proper enable control
  
  Serial.println("=== MOTOR TEST COMPLETE ===");
  
  // Ensure motors are ready for normal operation
  Serial.println("Motors ready for normal operation");
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
  Serial.println("🔧 MOTOR DEBUG: Setting clockwise rotation with speed " + String(speed));
  Serial.println("🔧 MOTOR DEBUG: Left motor forward=HIGH, backward=LOW");
  Serial.println("🔧 MOTOR DEBUG: Right motor forward=LOW, backward=HIGH");
  
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
  analogWrite(MOTOR_LEFT_ENABLE, speed);
  analogWrite(MOTOR_RIGHT_ENABLE, speed);
  
  Serial.println("🔧 MOTOR DEBUG: Motor pins set, enable pins written with PWM");
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
    case 1: // Lower arms (synchronized movement)
      if (millis() - stepStartTime >= 500) {
        moveArmsSynchronized(20, 20); // Both arms move DOWN together (Left=20°, Right=130°)
        Serial.println("COLLECTION: Arms lowered synchronized: Left=20° (front-down), Right=130° (back-down)");
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
      
    case 4: // Lift arms (synchronized movement)
      if (millis() - stepStartTime >= 1000) {
        moveArmsSynchronized(150, 150); // Both arms move UP together (Left=150°, Right=0°)
        Serial.println("COLLECTION: Arms lifted synchronized: Left=150° (front-high), Right=0° (back-high)");
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
      
    case 6: // Open ramp (0° to 90°) - SG90 servo
      if (millis() - stepStartTime >= 500) {
        moveRampServoSafely(90);
        Serial.println("COLLECTION: Ramp opened");
        stepStartTime = millis();
        collectionStep = 7;
      }
      break;
      
    case 7: // Close ramp (90° to 0°) - SG90 servo
      if (millis() - stepStartTime >= 2000) {
        moveRampServoSafely(0);
        Serial.println("COLLECTION: Ramp closed");
        stepStartTime = millis();
        collectionStep = 8;
      }
      break;
      
    case 8: // Reset arms (synchronized movement)
      if (millis() - stepStartTime >= 500) {
        moveArmsSynchronized(60, 60); // Both arms move to initial position (Left=60°, Right=90°)
        Serial.println("COLLECTION: Arms reset synchronized: Left=60° (front-up), Right=90° (back-up)");
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
void resetToInitialPositionGently() {
  Serial.println("Gently moving servos to initial positions...");
  
  // SG92R Arms: Move SIMULTANEOUSLY to parallel positions (attached to same box)
  Serial.println("Moving SG92R arm servos TOGETHER to parallel positions...");
  
  // Both arms start calibration movement at EXACTLY the same time
  Serial.println("Both arms starting synchronized movement from 90° to final positions...");
  for (int pos = 90; pos >= 60; pos--) {
    servoArmLeft.write(pos);           // Left arm moves DOWN (front-facing servo: 90°->60°)
    servoArmRight.write(150 - pos);    // Right arm moves DOWN synchronized (back-facing servo: 60°->90°)
    delay(100); // Slower for SG92R stability and synchronized movement
  }
  
  armLeftPosition = 60;
  armRightPosition = 90; // Right servo at 90° (150°-60°) for synchronized movement
  Serial.println("Arms positioned PARALLEL simultaneously: Both moved in SAME DIRECTION (Left=60°, Right=90°)");
  
  // Box servos (SG92R): slowly move to 0 degrees (open)
  Serial.println("Moving box servos...");
  for (int pos = 45; pos >= 0; pos--) {
    servoBoxLeft.write(pos);
    servoBoxRight.write(pos);
    delay(80);
  }
  boxLeftPosition = 0;
  boxRightPosition = 0;
  Serial.println("Box servos opened (0°)");
  
  // SG90 Ramp: Special handling to prevent continuous rotation
  Serial.println("Positioning SG90 ramp servo...");
  delay(500); // Extra delay before SG90 movement
  
  // For SG90, move more slowly and with pauses
  for (int pos = 90; pos >= 0; pos -= 5) {
    servoRamp.write(pos);
    delay(200); // Much slower for SG90
  }
  servoRamp.write(0); // Final position
  rampPosition = 0;
  Serial.println("Ramp closed (0°)");
  
  delay(1000); // Final stabilization
  Serial.println("All servos gently positioned - SG92R arms stable, SG90 ramp controlled");
}

void resetToInitialPosition() {
  // Set arms to parallel positions (both facing different directions but same angle)
  servoArmLeft.write(60);   // Left arm to 60° (front-facing, up)
  servoArmRight.write(60);  // Right arm to 60° (back-facing, up)
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

// Special function for SG90 ramp servo to prevent continuous rotation
void moveRampServoSafely(int targetPos) {
  Serial.println("Moving SG90 ramp servo from " + String(rampPosition) + "° to " + String(targetPos) + "°");
  
  // For SG90, use slower movement and verification
  int currentPos = rampPosition;
  int step = (targetPos > currentPos) ? 5 : -5;
  
  if (currentPos != targetPos) {
    for (int pos = currentPos; abs(pos - targetPos) > 5; pos += step) {
      servoRamp.write(pos);
      delay(150); // Slower for SG90
    }
  }
  
  // Final position with confirmation
  servoRamp.write(targetPos);
  delay(300);
  servoRamp.write(targetPos); // Double-write for SG90 stability
  rampPosition = targetPos;
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

// Synchronized arm movement (for opposite-mounted servos)
void moveArmsSynchronized(int leftTarget, int rightSyncTarget) {
  // Left servo moves normally, right servo moves in adjusted reverse for synchronization
  int rightTarget = 150 - rightSyncTarget; // Adjusted calculation: 150° instead of 180°
  
  int maxSteps = max(abs(leftTarget - armLeftPosition), abs(rightTarget - armRightPosition));
  
  for (int i = 0; i <= maxSteps; i++) {
    int leftPos = armLeftPosition + (leftTarget - armLeftPosition) * i / maxSteps;
    int rightPos = armRightPosition + (rightTarget - armRightPosition) * i / maxSteps;
    
    servoArmLeft.write(leftPos);
    servoArmRight.write(rightPos);
    delay(15);
  }
  
  servoArmLeft.write(leftTarget);
  servoArmRight.write(rightTarget);
  
  armLeftPosition = leftTarget;
  armRightPosition = rightTarget;
}


