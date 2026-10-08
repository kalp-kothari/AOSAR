#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

#define NUM_MOTORS 3

// Encoder pins
const int encoderA[NUM_MOTORS] = {32, 25, 27};
const int encoderB[NUM_MOTORS] = {33, 26, 14};

// Encoder counts
extern volatile long encoderCount[NUM_MOTORS];

void encoderInit();

long getEncoderCount(int motor);

void resetEncoders();

#endif