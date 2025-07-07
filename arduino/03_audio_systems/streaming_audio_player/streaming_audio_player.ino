/*
 * Arduino Streaming Audio Player
 * Receives 8-bit PCM audio data via serial and plays through PWM
 * Based on proven Arduino audio streaming implementations
 * 
 * Hardware:
 * - Audio output on pin 9 (PWM)
 * - RC filter: 2.7kΩ resistor + 10nF capacitor to GND
 * - Amplifier input connected to filtered output
 * 
 * Audio Format:
 * - 8-bit unsigned PCM
 * - 8kHz sample rate
 * - Mono channel
 * - Serial at 115200 baud
 * 
 * Based on: 
 * - https://github.com/idanre1/audioStreamArduino
 * - Arduino Audio Tools serial streaming
 * - Proven PCM audio implementations
 */

#include <avr/interrupt.h>
#include <avr/io.h>

// Audio configuration
#define SAMPLE_RATE 8000        // 8kHz sample rate
#define AUDIO_PIN 9             // PWM output pin
#define BUFFER_SIZE 128         // Buffer size for each ping-pong buffer
#define TRANSFER_SIZE 64        // Chunk size for serial transfer

// Ping-pong buffers
volatile uint8_t audioBuffer1[BUFFER_SIZE];
volatile uint8_t audioBuffer2[BUFFER_SIZE];
volatile uint8_t* currentBuffer = audioBuffer1;
volatile uint8_t* fillBuffer = audioBuffer2;

// Buffer management
volatile uint16_t playIndex = 0;
volatile uint16_t fillIndex = 0;
volatile uint16_t samplesInBuffer = 0;
volatile bool bufferReady = false;
volatile bool isPlaying = false;

// Serial communication
volatile bool waitingForData = false;
volatile uint8_t expectedBytes = 0;

// Commands
#define CMD_PLAY 'P'
#define CMD_STOP 'S'
#define CMD_TRANSFER_SIZE 'T'
#define CMD_GO 'G'
#define RESPONSE_GO "GO"
#define RESPONSE_READY "READY"

void setup() {
  Serial.begin(115200);
  
  // Configure PWM output pin
  pinMode(AUDIO_PIN, OUTPUT);
  
  // Initialize timer for PWM
  setupPWM();
  
  // Initialize timer for sample rate
  setupSampleTimer();
  
  // Initialize buffers
  memset((void*)audioBuffer1, 127, BUFFER_SIZE);  // Mid-level silence
  memset((void*)audioBuffer2, 127, BUFFER_SIZE);
  
  // Send initial ready signal
  Serial.println(RESPONSE_GO);
  Serial.flush();
}

void setupPWM() {
  // Configure Timer1 for fast PWM mode
  // PWM frequency = 16MHz / (1 * 256) = 62.5kHz
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);  // Fast PWM 8-bit
  TCCR1B = _BV(WGM12) | _BV(CS10);                  // No prescaling
  OCR1A = 127;  // Initial mid-level output
}

void setupSampleTimer() {
  // Configure Timer2 for sample rate timing
  // Timer2 interrupt at 8kHz = 16MHz / (1 * 256 * 8) = 7812.5Hz ≈ 8kHz
  TCCR2A = _BV(WGM21);           // CTC mode
  TCCR2B = _BV(CS22);            // Prescaler 64
  OCR2A = 249;                   // 16MHz / (64 * 250) = 8kHz
  TIMSK2 = _BV(OCIE2A);          // Enable interrupt
}

// Timer2 interrupt - called at sample rate (8kHz)
ISR(TIMER2_COMPA_vect) {
  if (isPlaying && samplesInBuffer > 0) {
    // Output current sample to PWM
    OCR1A = currentBuffer[playIndex];
    
    // Move to next sample
    playIndex++;
    samplesInBuffer--;
    
    // Check if we need to swap buffers
    if (playIndex >= BUFFER_SIZE) {
      playIndex = 0;
      
      // Swap buffers
      volatile uint8_t* temp = currentBuffer;
      currentBuffer = fillBuffer;
      fillBuffer = temp;
      
      // Reset for next buffer
      samplesInBuffer = fillIndex;
      fillIndex = 0;
      bufferReady = false;
      waitingForData = true;
    }
  } else if (isPlaying) {
    // No more samples - output silence
    OCR1A = 127;
    isPlaying = false;
  }
}

void loop() {
  handleSerialCommands();
  
  // Request more data if needed
  if (waitingForData && !bufferReady) {
    Serial.println(RESPONSE_READY);
    Serial.flush();
    waitingForData = false;
  }
}

void handleSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case CMD_GO:
        // Handshake response
        Serial.println(RESPONSE_GO);
        Serial.flush();
        break;
        
      case CMD_TRANSFER_SIZE:
        // Send our preferred transfer size
        Serial.println(TRANSFER_SIZE);
        Serial.flush();
        break;
        
      case CMD_PLAY:
        // Start playing - expect audio data
        isPlaying = true;
        waitingForData = true;
        expectedBytes = TRANSFER_SIZE;
        playIndex = 0;
        fillIndex = 0;
        samplesInBuffer = 0;
        break;
        
      case CMD_STOP:
        // Stop playing
        isPlaying = false;
        OCR1A = 127;  // Silence
        break;
        
      default:
        // Assume it's audio data
        if (isPlaying && fillIndex < BUFFER_SIZE) {
          fillBuffer[fillIndex++] = (uint8_t)command;
          expectedBytes--;
          
          // Check if we have enough data to start/continue
          if (fillIndex >= TRANSFER_SIZE) {
            if (samplesInBuffer == 0) {
              // First data received, start playing
              samplesInBuffer = fillIndex;
              fillIndex = 0;
              
              // Swap buffers
              volatile uint8_t* temp = currentBuffer;
              currentBuffer = fillBuffer;
              fillBuffer = temp;
            }
            bufferReady = true;
            waitingForData = false;
          }
        }
        break;
    }
  }
}

void serialEvent() {
  // This function is called automatically when serial data arrives
  // We handle it in the main loop for better control
} 