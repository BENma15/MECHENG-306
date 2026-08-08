#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <avr/interrupt.h>

// Left Motor Encoder Pins
extern const int L_ENCA; // INT3
extern const int L_ENCB; // INT2

// Right Motor eEncoder Pins
extern const int R_ENCA; // INT1
extern const int R_ENCB; // INT0

// Current Encoder Counts
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

void Encoder_Init();
void Encoder_updateLeft();
void Encoder_updateRight();
long Encoder_getLeftEncoderCount();
long Encoder_getRightEncoderCount();

#endif