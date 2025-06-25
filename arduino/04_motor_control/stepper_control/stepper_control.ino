/*
 * Solar Shrine - Stepper Motor Control with Sensor Integration
 * Controls stepper motor based on ultrasonic sensor readings
 * 
 * Pin Configuration:
 * - Pins 2,3,4,5: Stepper motor control (adjust as needed)
 * - Pins 9,10: Ultrasonic sensor 1 (Trig, Echo)
 * - Pins 5,6: Ultrasonic sensor 2 (Trig, Echo) - Note: Pin 5 shared with stepper
 * 
 * Features:
 * - Sensor-driven motor positioning
 * - Serial output for monitoring
 * - Smooth motor movement
 */

#include <Stepper.h>

// Stepper motor configuration
const int stepsPerRevolution = 200;  // Adjust for your stepper motor
const int motorSpeed = 100;          // RPM
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 7); // Using pins 2,3,4,7 for stepper

// Sensor pins (adjusted to avoid stepper conflicts)
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 8;   // Changed from 5 to avoid stepper conflict
const int echoPin2 = 6;

// Motor control variables
int currentPosition = 0;
int targetPosition = 0;
const int maxSteps = 400;  // Maximum steps in either direction
const int stepIncrement = 10;

// Distance mapping
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Stepper setup
  myStepper.setSpeed(motorSpeed);
  
  Serial.println("Solar Shrine - Stepper Motor Control");
  Serial.println("Sensor-driven positioning system");
  
  // Home the motor (optional - center position)
  Serial.println("Homing motor to center position...");
  currentPosition = 0;
  targetPosition = 0;
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  if (distance <= 0 || duration == 0) {
    return MAX_RANGE + 1;
  }
  
  return distance;
}

int mapDistanceToSteps(float distance) {
  if (distance < MIN_RANGE || distance > MAX_RANGE) {
    return currentPosition; // No change if out of range
  }
  
  // Map distance to stepper position
  float ratio = (distance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
  int steps = -maxSteps + (int)(2 * maxSteps * ratio);
  
  return constrain(steps, -maxSteps, maxSteps);
}

void moveToPosition(int newPosition) {
  int stepsToMove = newPosition - currentPosition;
  
  if (stepsToMove != 0) {
    Serial.print("Moving motor ");
    Serial.print(abs(stepsToMove));
    Serial.print(" steps ");
    Serial.println(stepsToMove > 0 ? "clockwise" : "counter-clockwise");
    
    myStepper.step(stepsToMove);
    currentPosition = newPosition;
    
    Serial.print("Motor position: ");
    Serial.println(currentPosition);
  }
}

void smoothMoveToPosition(int newPosition) {
  while (currentPosition != newPosition) {
    int stepsToMove = newPosition - currentPosition;
    
    // Move in smaller increments for smoother motion
    if (abs(stepsToMove) > stepIncrement) {
      stepsToMove = (stepsToMove > 0) ? stepIncrement : -stepIncrement;
    }
    
    myStepper.step(stepsToMove);
    currentPosition += stepsToMove;
    
    // Small delay for smooth movement
    delay(50);
  }
}

void runPositionTest() {
  Serial.println("Running position test sequence...");
  
  // Test positions
  int testPositions[] = {0, 100, -100, 200, -200, 0};
  int numPositions = sizeof(testPositions) / sizeof(testPositions[0]);
  
  for (int i = 0; i < numPositions; i++) {
    Serial.print("Moving to position: ");
    Serial.println(testPositions[i]);
    
    smoothMoveToPosition(testPositions[i]);
    delay(2000); // Hold position for 2 seconds
  }
  
  Serial.println("Position test complete");
}

void runSensorReactiveMode() {
  // Read both sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Use the closer sensor for control
  float activeDistance = min(distance1, distance2);
  
  if (activeDistance >= MIN_RANGE && activeDistance <= MAX_RANGE) {
    int newTarget = mapDistanceToSteps(activeDistance);
    
    // Only move if target changed significantly
    if (abs(newTarget - targetPosition) > 5) {
      targetPosition = newTarget;
      
      Serial.print("Distance: ");
      Serial.print(activeDistance);
      Serial.print(" cm -> Target: ");
      Serial.println(targetPosition);
    }
  }
  
  // Move towards target position smoothly
  if (currentPosition != targetPosition) {
    int step = (targetPosition > currentPosition) ? 1 : -1;
    myStepper.step(step);
    currentPosition += step;
  }
}

void printStatus() {
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  Serial.print("Sensor 1: ");
  Serial.print(distance1);
  Serial.print(" cm, Sensor 2: ");
  Serial.print(distance2);
  Serial.print(" cm, Motor Position: ");
  Serial.print(currentPosition);
  Serial.print("/");
  Serial.println(maxSteps);
}

void printMenu() {
  Serial.println("\n=== Solar Shrine Motor Control Menu ===");
  Serial.println("1. Position Test Sequence");
  Serial.println("2. Sensor Reactive Mode (30 seconds)");
  Serial.println("3. Manual Position Control");
  Serial.println("4. Home Motor (center position)");
  Serial.println("5. Status Report");
  Serial.println("Enter choice (1-5):");
}

void loop() {
  static unsigned long lastMenuTime = 0;
  static unsigned long lastStatusTime = 0;
  unsigned long currentTime = millis();
  
  // Print menu every 15 seconds
  if (currentTime - lastMenuTime > 15000) {
    printMenu();
    lastMenuTime = currentTime;
  }
  
  // Print status every 5 seconds in reactive mode
  if (currentTime - lastStatusTime > 5000) {
    printStatus();
    lastStatusTime = currentTime;
  }
  
  // Check for serial input
  if (Serial.available() > 0) {
    int choice = Serial.parseInt();
    
    switch (choice) {
      case 1:
        runPositionTest();
        break;
        
      case 2:
        Serial.println("Sensor reactive mode - 30 seconds");
        for (int i = 0; i < 600; i++) { // 30 seconds at 50ms intervals
          runSensorReactiveMode();
          delay(50);
        }
        Serial.println("Reactive mode complete");
        break;
        
      case 3:
        Serial.println("Manual position control");
        Serial.println("Enter target position (-400 to 400):");
        while (!Serial.available()) {
          delay(100);
        }
        int manualTarget = Serial.parseInt();
        manualTarget = constrain(manualTarget, -maxSteps, maxSteps);
        Serial.print("Moving to position: ");
        Serial.println(manualTarget);
        smoothMoveToPosition(manualTarget);
        break;
        
      case 4:
        Serial.println("Homing motor to center...");
        smoothMoveToPosition(0);
        Serial.println("Motor homed");
        break;
        
      case 5:
        printStatus();
        break;
        
      default:
        if (choice != 0) {
          Serial.println("Invalid choice. Please enter 1-5.");
        }
        break;
    }
  }
  
  delay(100);
} 