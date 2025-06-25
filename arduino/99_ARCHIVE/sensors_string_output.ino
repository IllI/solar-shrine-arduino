/*
 * Dual Ultrasonic Sensor Reader
 */

// Sensor pins
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// Range constants (in cm)
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;
}

void loop() {
  // Read sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Print both sensor values as space-separated integers
  Serial.print(int(distance1));
  Serial.print(" ");
  Serial.println(int(distance2));
  
  delay(10);
} 
