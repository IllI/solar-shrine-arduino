/*
 * Solar Shrine - HC-SR04 Reset Fix Test
 * 
 * Testing for the "stuck echo" problem with HC-SR04 sensors
 * Some HC-SR04 sensors get stuck in HIGH state when no echo is detected
 * This version includes the fix from mc-computing.com
 * 
 * Pins: 10 (Trig), 11 (Echo) - corrected by user
 */

const int trigPin = 10; // User corrected this
const int echoPin = 11; // User corrected this

const unsigned long SENSOR_TIMEOUT = 50000UL; // 50ms timeout

void setup() {
  Serial.begin(9600);
  while (!Serial);
  delay(500);

  Serial.println("=== HC-SR04 RESET FIX TEST ===");
  Serial.print("Trigger Pin: ");
  Serial.println(trigPin);
  Serial.print("Echo Pin: ");
  Serial.println(echoPin);
  Serial.println("================================");
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.println("Testing for HC-SR04 'stuck echo' problem...");
  Serial.println("This version includes the fix for sensors that don't auto-reset");
  Serial.println("\nStarting sensor readings...\n");
}

float readDistanceWithReset() {
  // Check if echo pin is stuck HIGH from previous failed reading
  if (digitalRead(echoPin) == HIGH) {
    Serial.print("  [RESET] Echo pin stuck HIGH - applying fix... ");
    
    // Apply the fix: briefly drive echo pin LOW to reset sensor
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(10);
    pinMode(echoPin, INPUT);
    delayMicroseconds(10);
    
    // Verify the fix worked
    if (digitalRead(echoPin) == LOW) {
      Serial.println("SUCCESS!");
    } else {
      Serial.println("FAILED - sensor may be defective");
      return -1;
    }
  }
  
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for echo
  unsigned long duration = pulseIn(echoPin, HIGH, SENSOR_TIMEOUT);
  
  if (duration == 0) {
    return 0; // No echo detected
  }
  
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

void loop() {
  static int readingCount = 0;
  readingCount++;
  
  Serial.print("Reading #");
  Serial.print(readingCount);
  Serial.print(": ");

  float distance = readDistanceWithReset();
  
  if (distance == -1) {
    Serial.println("SENSOR ERROR - Reset failed");
  } else if (distance == 0) {
    Serial.println("TIMEOUT - No echo (normal for distant/missing objects)");
  } else {
    Serial.print("SUCCESS! Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(500);
} 