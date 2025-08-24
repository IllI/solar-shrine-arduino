#line 1 "C:\\Users\\cityz\\IllI\\play\\soulShine\\solar-shrine-arduino\\arduino\\01_MAIN_SYSTEM\\solar_shrine_playa\\ThereminEffect.cpp"
#include "ThereminEffect.h"

ThereminEffect::ThereminEffect() : aOscil(SIN2048_DATA), aVibrato(SIN2048_DATA) {
    // Constructor
}

void ThereminEffect::setup() {
    theremin_freq = 0;
    theremin_vol = 0;
    aVibrato.setFreq(5.f);
}

void ThereminEffect::enter() {
    // Enter logic
}

void ThereminEffect::exit() {
    // Exit logic
}

void ThereminEffect::update(int rightHand, int leftHand, int d1, int d2) {
    if (rightHand) {
        theremin_freq = map(d2, 1, 20, 1000, 200);
    } else {
        theremin_freq = 0;
    }

    if (leftHand) {
        theremin_vol = map(d1, 1, 20, 255, 0);
    } else {
        theremin_vol = 0;
    }

    aOscil.setFreq(theremin_freq + aVibrato.next());
}

int ThereminEffect::audio() {
    return (aOscil.next() * theremin_vol) >> 8;
}
