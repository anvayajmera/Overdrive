#include <Arduino.h>

// All-motors idle test: force every motor pin LOW and do nothing else.
// Use this to check for noise at idle.

static const uint8_t MOTOR_IN_PINS[] = {
    17, 21, // front left
    27, 14, // front right
    13, 18, // back right
    25, 19  // back left
};

static const uint8_t MOTOR_PIN_COUNT = sizeof(MOTOR_IN_PINS) / sizeof(MOTOR_IN_PINS[0]);

void setup()
{
    for (uint8_t i = 0; i < MOTOR_PIN_COUNT; i++)
    {
        pinMode(MOTOR_IN_PINS[i], OUTPUT);
        digitalWrite(MOTOR_IN_PINS[i], LOW);
    }
}

void loop()
{
    // Keep everything idle.
    delay(1000);
}
