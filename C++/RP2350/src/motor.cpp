#include "motor.hpp"
#include <Arduino.h>

Motor::Motor(int IN1, int IN2) : IN1(IN1), IN2(IN2) {}

void Motor::setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    Motor::speed = 0;
    digitalWrite(IN2, LOW);
}

void Motor::setSpeed(int speed)
{
    Motor::speed = (int)(((double)speed / 100) * 255);
    analogWrite(IN1, Motor::speed);
}

void Motor::stop()
{
    setSpeed(0);
}

void Motor::reverse()
{
    analogWrite(IN1, 0);
    digitalWrite(IN2, HIGH);
    analogWrite(IN1, Motor::speed);
}
