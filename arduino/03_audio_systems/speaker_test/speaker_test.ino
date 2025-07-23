/*
 * Solar Shrine - Speaker Test & Audio Integration
 * Tests the WWZMDiB XH-M543 amplifier with Dayton Audio DAEX32QMB-4 exciter
 * 
 * Pin Configuration:
 * - Pin 11: Audio output to amplifier
 * - Pins 5,6,9,10: Ultrasonic sensors (optional for audio-reactive testing)
 * 
 * Hardware:
 * - WWZMDiB XH-M543 High Power Digital Amplifier Board TPA3116D2
 * - Dayton Audio DAEX32QMB-4 Quad Feet Mega Bass 32mm Exciter 40W 4 Ohm
 */

// Audio pin
const int AUDIO_PIN = 11;

// Sensor pins (optional for audio-reactive testing)
const int trigPin1 = 9;
const int echoPin1 = 10;
const int trigPin2 = 5;
const int echoPin2 = 6;

// Test variables
unsigned long lastTestTime = 0;
int testPhase = 0;
const unsigned long TEST_INTERVAL = 3000; // 3 seconds between tests

// Frequency ranges for testing
const int TEST_FREQUENCIES[] = {
  220,   // A3
  440,   // A4
  880,   // A5
  1760,  // A6
  330,   // E4
  659,   // E5
  110,   // A2 (bass)
  2000   // High frequency
};
const int NUM_TEST_FREQS = sizeof(TEST_FREQUENCIES) / sizeof(TEST_FREQUENCIES[0]);

// Distance-based audio settings
const float MIN_RANGE = 1.0;
const float MAX_RANGE = 20.0;
const float MIN_FREQ = 80.0;    // Low bass
const float MAX_FREQ = 2000.0;  // High treble

void setup() {
  Serial.begin(9600);
  delay(1000);
  
  // Sensor setup (optional)
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  Serial.println("Solar Shrine - Speaker Test Starting");
  Serial.println("Hardware: WWZMDiB XH-M543 + Dayton Audio DAEX32QMB-4");
  Serial.println("Testing audio output on pin 11");
  
  // Play startup sequence
  playStartupSequence();
}

void playStartupSequence() {
  Serial.println("Playing startup sequence...");
  
  // Play ascending notes
  int melody[] = {220, 247, 277, 294, 330, 370, 415, 440}; // A3 to A4 chromatic
  for (int i = 0; i < 8; i++) {
    tone(AUDIO_PIN, melody[i]);
    delay(200);
  }
  
  // Short silence
  noTone(AUDIO_PIN);
  delay(500);
  
  // Play descending notes
  for (int i = 7; i >= 0; i--) {
    tone(AUDIO_PIN, melody[i]);
    delay(150);
  }
  
  noTone(AUDIO_PIN);
  Serial.println("Startup sequence complete");
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

void runFrequencyTest() {
  Serial.print("Testing frequency: ");
  Serial.print(TEST_FREQUENCIES[testPhase]);
  Serial.println(" Hz");
  
  // Play test tone for 2 seconds
  tone(AUDIO_PIN, TEST_FREQUENCIES[testPhase]);
  delay(2000);
  noTone(AUDIO_PIN);
  
  // Short silence
  delay(500);
  
  // Move to next test frequency
  testPhase = (testPhase + 1) % NUM_TEST_FREQS;
  
  if (testPhase == 0) {
    Serial.println("Frequency test cycle complete");
    delay(2000);
  }
}

void runDistanceReactiveTest() {
  // Read both sensors
  float distance1 = readDistance(trigPin1, echoPin1);
  float distance2 = readDistance(trigPin2, echoPin2);
  
  // Use the closer of the two sensors
  float activeDistance = min(distance1, distance2);
  
  if (activeDistance >= MIN_RANGE && activeDistance <= MAX_RANGE) {
    // Map distance to frequency
    float ratio = (activeDistance - MIN_RANGE) / (MAX_RANGE - MIN_RANGE);
    ratio = constrain(ratio, 0.0, 1.0);
    
    int frequency = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (1.0 - ratio);
    
    tone(AUDIO_PIN, frequency);
    
    Serial.print("Distance: ");
    Serial.print(activeDistance);
    Serial.print(" cm, Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");
  } else {
    noTone(AUDIO_PIN);
  }
}

void runSweepTest() {
  Serial.println("Running frequency sweep test...");
  
  // Sweep up
  for (int freq = 100; freq <= 2000; freq += 20) {
    tone(AUDIO_PIN, freq);
    delay(50);
  }
  
  // Sweep down
  for (int freq = 2000; freq >= 100; freq -= 20) {
    tone(AUDIO_PIN, freq);
    delay(50);
  }
  
  noTone(AUDIO_PIN);
  Serial.println("Sweep test complete");
}

void runRhythmTest() {
  Serial.println("Running rhythm test...");
  
  int rhythm[] = {440, 0, 440, 0, 880, 0, 440, 0, 660, 0, 440, 0};
  int durations[] = {200, 100, 200, 100, 400, 200, 200, 100, 300, 200, 400, 500};
  
  for (int i = 0; i < 12; i++) {
    if (rhythm[i] > 0) {
      tone(AUDIO_PIN, rhythm[i]);
    } else {
      noTone(AUDIO_PIN);
    }
    delay(durations[i]);
  }
  
  noTone(AUDIO_PIN);
  Serial.println("Rhythm test complete");
}

void runHarmonicTest() {
  Serial.println("Running harmonic test...");
  
  // Test fundamental and harmonics
  int fundamental = 220; // A3
  int harmonics[] = {220, 440, 660, 880, 1100, 1320}; // Fundamental + 5 harmonics
  
  for (int i = 0; i < 6; i++) {
    Serial.print("Playing harmonic ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(harmonics[i]);
    Serial.println(" Hz");
    
    tone(AUDIO_PIN, harmonics[i]);
    delay(1000);
  }
  
  noTone(AUDIO_PIN);
  Serial.println("Harmonic test complete");
}

void printTestMenu() {
  Serial.println("\n=== Solar Shrine Audio Test Menu ===");
  Serial.println("1. Frequency Test (cycles through test frequencies)");
  Serial.println("2. Distance Reactive Test (requires sensor input)");
  Serial.println("3. Frequency Sweep Test");
  Serial.println("4. Rhythm Test");
  Serial.println("5. Harmonic Test");
  Serial.println("6. Startup Sequence");
  Serial.println("7. Stop Audio");
  Serial.println("Enter test number (1-7):");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Print menu every 10 seconds if no input
  static unsigned long lastMenuTime = 0;
  if (currentTime - lastMenuTime > 10000) {
    printTestMenu();
    lastMenuTime = currentTime;
  }
  
  // Check for serial input
  if (Serial.available() > 0) {
    int testChoice = Serial.parseInt();
    
    switch (testChoice) {
      case 1:
        runFrequencyTest();
        break;
      case 2:
        Serial.println("Distance reactive mode - move hands near sensors");
        for (int i = 0; i < 100; i++) { // Run for ~5 seconds
          runDistanceReactiveTest();
          delay(50);
        }
        noTone(AUDIO_PIN);
        break;
      case 3:
        runSweepTest();
        break;
      case 4:
        runRhythmTest();
        break;
      case 5:
        runHarmonicTest();
        break;
      case 6:
        playStartupSequence();
        break;
      case 7:
        noTone(AUDIO_PIN);
        Serial.println("Audio stopped");
        break;
      default:
        if (testChoice != 0) {
          Serial.println("Invalid choice. Please enter 1-7.");
        }
        break;
    }
  }
  
  // Auto-test mode - cycle through tests every 30 seconds if no input
  if (currentTime - lastTestTime > 30000) {
    if (!Serial.available()) {
      Serial.println("Auto-test mode: Running frequency test");
      runFrequencyTest();
    }
    lastTestTime = currentTime;
  }
  
  delay(100);
} 