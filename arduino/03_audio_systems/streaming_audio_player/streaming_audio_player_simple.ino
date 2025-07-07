/*
 * Simplified Arduino Streaming Audio Player
 * Based on forum research and successful implementations
 * Focuses on reliability over complexity
 * 
 * Key fixes from research:
 * - Simpler buffer management (no ping-pong)
 * - Lower data rate for reliability  
 * - Immediate audio output
 * - Simplified protocol
 * 
 * Hardware:
 * - Audio output on pin 9 (PWM)
 * - RC filter: 2.7kΩ resistor + 10nF capacitor to GND
 */

// Simple audio buffer
#define BUFFER_SIZE 64
volatile uint8_t audioBuffer[BUFFER_SIZE];
volatile uint8_t bufferIndex = 0;
volatile uint8_t playIndex = 0;
volatile bool hasData = false;

// Audio output
#define AUDIO_PIN 9

void setup() {
  Serial.begin(115200);
  
  // Configure PWM pin
  pinMode(AUDIO_PIN, OUTPUT);
  
  // Setup PWM (Timer1 for pin 9)
  // Fast PWM, 8-bit, ~31kHz frequency
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);  // No prescaler
  OCR1A = 127;  // Mid-level
  
  // Setup timer for sample rate (Timer2)
  TCCR2A = _BV(WGM21);  // CTC mode
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler 256
  OCR2A = 124;  // ~4kHz sample rate (16MHz/(256*125) = 500Hz * 8 = 4kHz)
  TIMSK2 = _BV(OCIE2A);  // Enable interrupt
  
  // Initialize buffer with silence
  for (int i = 0; i < BUFFER_SIZE; i++) {
    audioBuffer[i] = 127;  // Mid-level (silence)
  }
  
  Serial.println("READY");
  Serial.flush();
}

// Audio interrupt - plays samples at ~4kHz
ISR(TIMER2_COMPA_vect) {
  if (hasData) {
    OCR1A = audioBuffer[playIndex];
    playIndex = (playIndex + 1) % BUFFER_SIZE;
    
    // If we've caught up to write position, we're out of data
    if (playIndex == bufferIndex) {
      hasData = false;
      OCR1A = 127;  // Output silence
    }
  } else {
    OCR1A = 127;  // Output silence
  }
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'G') {
      // Handshake
      Serial.println("GO");
      Serial.flush();
      
    } else if (command == 'T') {
      // Transfer size query
      Serial.println("32");  // Small chunks for reliability
      Serial.flush();
      
    } else if (command == 'P') {
      // Play command - reset and start
      bufferIndex = 0;
      playIndex = 0;
      hasData = false;
      Serial.println("READY");
      Serial.flush();
      
    } else if (command == 'S') {
      // Stop command
      hasData = false;
      OCR1A = 127;
      
    } else {
      // Audio data - store in buffer
      audioBuffer[bufferIndex] = (uint8_t)command;
      bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
      
      // Start playing when we have some data
      if (!hasData && bufferIndex >= 8) {  // Start with small buffer
        hasData = true;
      }
      
      // Send ready signal when buffer has space
      if ((bufferIndex + 1) % BUFFER_SIZE != playIndex) {
        // Buffer not full, ready for more
        if (bufferIndex % 16 == 0) {  // Every 16 bytes
          Serial.println("READY");
          Serial.flush();
        }
      }
    }
  }
} 