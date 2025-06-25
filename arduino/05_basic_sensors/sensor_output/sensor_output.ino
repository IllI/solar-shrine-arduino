/*
 * Solar Shrine - Basic Sensor Output
 * Simple ultrasonic sensor reading with serial output
 * 
 * Pin Configuration:
 * - Pins 9,10: Ultrasonic sensor 1 (Trig, Echo) 
 * - Pins 5,6: Ultrasonic sensor 2 (Trig, Echo)
 * 
 * Features:
 * - Dual sensor reading
 * - Serial monitor output
 * - Basic distance measurement
 */

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  Serial.println("Solar Shrine - Basic Sensor Output");
  Serial.println("Reading dual ultrasonic sensors");
  Serial.println("Format: Sensor1_cm | Sensor2_cm");
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.0343) / 2;
  
  // Return 0 if reading seems invalid
  if (distance <= 0 || duration == 0) {
    return 0;
  }
  
  return distance;
}

void loop() {
  // Read both sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Output to serial monitor
  Serial.print(distance1);
  Serial.print(" | ");
  Serial.println(distance2);
  
  delay(100);  // 10Hz update rate
} 