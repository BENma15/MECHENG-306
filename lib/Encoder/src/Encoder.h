#ifndef ENCODER_H
#define ENCODER_H

#include <HelperFunctions.h>
#include <Arduino.h>
#include <avr/interrupt.h>

const long Y_MAX = 125;   //max distance y in counts
const long Y_MIN = 0;
const long X_MAX = 195;   //max distance x in counts
const long X_MIN = 0;

extern volatile long countA;
extern volatile long countB;

extern double Encoder_getLeftVelocity(double dt);
extern double Encoder_getRightVelocity(double dt);

// Left Motor Encoder Pins
extern const int L_ENCA; // INT3
extern const int L_ENCB; // INT2

// Right Motor eEncoder Pins
extern const int R_ENCA; // INT1
extern const int R_ENCB; // INT0

// Current Encoder Counts
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

extern double distance_per_encoder_tick;

void Encoder_Init();
void Encoder_updateLeft();
void Encoder_updateRight();
long Encoder_getLeftEncoderCount();
long Encoder_getRightEncoderCount();
void Encoder_setLeftEncoderCountZero();
void Encoder_setRightEncoderCountZero();
bool encoderCountsChanged();

#endif