#include "encoder.h"

volatile long encoderCount[NUM_MOTORS] = {0, 0, 0};


// Generic encoder handler
void IRAM_ATTR encoderHandler(int motor)
{
    int A = digitalRead(encoderA[motor]);
    int B = digitalRead(encoderB[motor]);

    if (A == B)
        encoderCount[motor]++;
    else
        encoderCount[motor]--;
}


void encoderInit()
{
    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pinMode(encoderA[i], INPUT_PULLUP);
        pinMode(encoderB[i], INPUT_PULLUP);

        attachInterruptArg(
            digitalPinToInterrupt(encoderA[i]),
            [](void *arg)
            {
                encoderHandler((int)(intptr_t)arg);
            },
            (void *)(intptr_t)i,
            CHANGE
        );
    }
}


long getEncoderCount(int motor)
{
    return encoderCount[motor];
}


void resetEncoders()
{
    for (int i = 0; i < NUM_MOTORS; i++)
        encoderCount[i] = 0;
}