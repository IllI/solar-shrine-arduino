#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\RobotsEffect.cpp"
#include "RobotsEffect.h"

RobotsEffect::RobotsEffect() : aSin(SIN2048_DATA), aSin2(SIN2048_DATA), aGain(SIN2048_DATA), aEnv() {
    // Constructor
}

void RobotsEffect::setup() {
    a = 1; b = 3; c = 5; d = 7; e = 9; f = 11; g = 13; h = 15;
    notes[0] = mtof(48 + a); notes[1] = mtof(48 + b); notes[2] = mtof(48 + c); notes[3] = mtof(48 + d);
    notes[4] = mtof(48 + e); notes[5] = mtof(48 + f); notes[6] = mtof(48 + g); notes[7] = mtof(48 + h);
    current_note_index = 0;
    note_change_time = 0;
    step = 0;
    aEnv.setAttackDecaySustainRelease(100, 100, 255, 100);
    aGain.setFreq(1.f);
}

void RobotsEffect::enter() {
    // Enter logic
}

void RobotsEffect::exit() {
    // Exit logic
}

void RobotsEffect::update(int rightHand, int leftHand, int d1, int d2) {
    if (millis() > note_change_time) {
        current_note_index = (current_note_index + 1) % 8;
        note_change_time = millis() + 250;
    }
    aSin.setFreq(notes[current_note_index]);
    aSin2.setFreq(notes[current_note_index] / 2);
    aEnv.update();
}

int RobotsEffect::audio() {
    if (step > 32766) {
        step = 0;
    }
    step++;
    if (step % 2048 < 1024) {
        aEnv.noteOn();
    } else {
        aEnv.noteOff();
    }
    long out = (long)(aSin.next() + aSin2.next()) * aEnv.next() >> 9;
    return (int)out;
}
