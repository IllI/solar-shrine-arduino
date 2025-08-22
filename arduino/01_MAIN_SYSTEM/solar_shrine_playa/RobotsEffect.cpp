#include "RobotsEffect.h"

#include <Oscil.h>
#include <tables/saw2048_int8.h>

namespace RobotsEffect {

  namespace {
    Oscil<SAW2048_NUM_CELLS, AUDIO_RATE> robotOsc(SAW2048_DATA);
    int robotPitchPattern[] = {220, 330, 440, 330, 220, 165, 220, 330};
    int robotPatternIndex = 0;
    unsigned long lastRobotChange = 0;
    const unsigned long ROBOT_NOTE_DURATION = 200;
    int robotVolume = 255; // Max volume
  }

  void setup() {
    // Setup is handled in enter()
  }

  void enter() {
    robotOsc.setFreq(robotPitchPattern[0]);
    lastRobotChange = millis();
  }

  void exit() {
    // No specific exit actions needed
  }

  void update() {
    if (millis() - lastRobotChange >= ROBOT_NOTE_DURATION) {
      robotPatternIndex = (robotPatternIndex + 1) % 8;
      robotOsc.setFreq(robotPitchPattern[robotPatternIndex]);
      lastRobotChange = millis();
    }
  }

  int audio() {
    return (robotOsc.next() * robotVolume);
  }

}