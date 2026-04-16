#include <Arduino.h>

// Single-motor pin pair test: IN1 = GPIO27, IN2 = GPIO14 (front right)
// This sketch ignores serial and just drives the pair forward/backward.
// Uses direct digital writes (no PWM) to remove PWM variables.

static const uint8_t IN1_PIN = 27;
static const uint8_t IN2_PIN = 14;
static void stopMotor()
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
}

static void driveForward()
{
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN1_PIN, HIGH);
}

static void driveBackward()
{
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
}

void setup()
{
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    stopMotor();
}

void loop()
{
    // Forward for 2 seconds, stop for 1, backward for 2, stop for 2
    driveForward();
    delay(2000);
    stopMotor();
    delay(1000);
    driveBackward();
    delay(2000);
    stopMotor();
    delay(2000);
}
