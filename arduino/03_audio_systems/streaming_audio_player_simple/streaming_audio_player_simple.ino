/*
 * Pre-Fill Buffer Arduino Audio Player
 * Eliminates buffer underruns by pre-loading data before playback
 * 
 * Key improvements:
 * - Pre-fill buffer before starting playback
 * - Larger buffer with safety margin
 * - Separate loading and playing phases
 * - Better PWM frequency for smoother audio
 * 
 * Hardware:
 * - Audio output on pin 9 (PWM)
 * - RC filter: 2kΩ (2x1kΩ) + 10nF capacitor to GND
 */

// Much larger buffer for pre-filling
#define BUFFER_SIZE 256
volatile uint8_t audioBuffer[BUFFER_SIZE];
volatile uint16_t bufferWriteIndex = 0;
volatile uint16_t bufferReadIndex = 0;
volatile bool isPlaying = false;
volatile bool bufferReady = false;

// Pre-fill threshold - wait until buffer is mostly full
#define PREFILL_THRESHOLD 192  // Fill 75% before starting

// Audio output
#define AUDIO_PIN 9

void setup() {
  Serial.begin(115200);
  
  // Configure PWM pin
  pinMode(AUDIO_PIN, OUTPUT);
  
  // Setup PWM with higher frequency for smoother audio
  // Timer1 Fast PWM, 8-bit, ~62kHz frequency (double normal)
  TCCR1A = _BV(COM1A1) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);  // No prescaler
  OCR1A = 127;  // Mid-level (silence)
  
  // Setup timer for 8kHz sample rate (Timer2)
  // More precise timing calculation
  TCCR2A = _BV(WGM21);  // CTC mode
  TCCR2B = _BV(CS22);   // Prescaler 64
  OCR2A = 30;           // 16MHz/(64*31) = ~8kHz
  TIMSK2 = _BV(OCIE2A); // Enable interrupt
  
  // Initialize buffer with silence
  for (int i = 0; i < BUFFER_SIZE; i++) {
    audioBuffer[i] = 127;  // Mid-level (silence)
  }
  
  Serial.println("READY");
  Serial.flush();
}

// Audio interrupt - only plays when buffer is ready
ISR(TIMER2_COMPA_vect) {
  if (isPlaying && bufferReady) {
    // Output current sample
    OCR1A = audioBuffer[bufferReadIndex];
    
    // Advance read pointer
    bufferReadIndex = (bufferReadIndex + 1) % BUFFER_SIZE;
    
    // Check buffer level - stop if getting too low
    uint16_t available = (bufferWriteIndex - bufferReadIndex + BUFFER_SIZE) % BUFFER_SIZE;
    if (available < 16) {  // Less than 16 samples left
      bufferReady = false;
      OCR1A = 127;  // Output silence
    }
  } else {
    OCR1A = 127;  // Output silence
  }
}

uint16_t getBufferLevel() {
  return (bufferWriteIndex - bufferReadIndex + BUFFER_SIZE) % BUFFER_SIZE;
}

void loop() {
  static bool isLoading = false;
  static uint16_t loadedSamples = 0;
  
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'G') {
      // Handshake
      Serial.println("GO");
      Serial.flush();
      
    } else if (command == 'T') {
      // Transfer size query - request larger chunks
      Serial.println("128");  // Even larger chunks
      Serial.flush();
      
    } else if (command == 'P') {
      // Play command - start loading phase
      cli();  // Disable interrupts
      bufferWriteIndex = 0;
      bufferReadIndex = 0;
      isPlaying = false;
      bufferReady = false;
      isLoading = true;
      loadedSamples = 0;
      sei();  // Enable interrupts
      
      Serial.println("LOADING");  // Different response to indicate loading
      Serial.flush();
      
    } else if (command == 'S') {
      // Stop command
      cli();  // Disable interrupts
      isPlaying = false;
      bufferReady = false;
      isLoading = false;
      OCR1A = 127;  // Silence
      sei();  // Enable interrupts
      
    } else if (isLoading) {
      // Audio data during loading phase
      audioBuffer[bufferWriteIndex] = (uint8_t)command;
      bufferWriteIndex = (bufferWriteIndex + 1) % BUFFER_SIZE;
      loadedSamples++;
      
      // Check if we've loaded enough to start playing
      if (loadedSamples >= PREFILL_THRESHOLD) {
        cli();  // Disable interrupts
        isPlaying = true;
        bufferReady = true;
        isLoading = false;
        sei();  // Enable interrupts
        
        Serial.println("PLAYING");  // Notify that playback started
        Serial.flush();
      }
      
      // Request more data if buffer not full
      if (loadedSamples % 64 == 0) {  // Every 64 samples
        Serial.println("MORE");
        Serial.flush();
      }
      
    } else if (isPlaying) {
      // Audio data during playback - only accept if buffer has space
      uint16_t available = getBufferLevel();
      if (available < (BUFFER_SIZE - 32)) {  // Leave some safety margin
        audioBuffer[bufferWriteIndex] = (uint8_t)command;
        bufferWriteIndex = (bufferWriteIndex + 1) % BUFFER_SIZE;
        
        // Re-enable buffer if it was disabled
        if (!bufferReady && getBufferLevel() >= 32) {
          bufferReady = true;
        }
      }
    }
  }
} 