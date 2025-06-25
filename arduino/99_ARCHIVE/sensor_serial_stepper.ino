This code will activate the motor and trigger effects in Touchdesigner when hands are covering the sensors:

/*
 * L298N Motor and Sensor Control
 */

// Motor pins - exact working configuration
const int IN3 = 4;
const int IN4 = 5;
const int ENB = 11;

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
  delay(2000);  // Same delay as working version
  
  // Motor setup - exactly as working version
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Sensor setup
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  // Start with motor off but ENB at full power
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255);  // Keep ENB at full power like working version
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
  
  bool inRange1 = (distance1 >= MIN_RANGE && distance1 <= MAX_RANGE);
  bool inRange2 = (distance2 >= MIN_RANGE && distance2 <= MAX_RANGE);
  
  if (inRange1 && inRange2) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    // Keep ENB at 255 like working version
    Serial.println("1");
  } else {
    // Use exact working motor configuration
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    // ENB already at 255 from setup
    Serial.println("0");
  }
  
  delay(10);
} 
