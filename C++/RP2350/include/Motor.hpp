#pragma once;

#include <Arduino.h>

class Motor
{
public:
    Motor();
    Motor(int IN1, int IN2);
    void setup();
    void setSpeed(int speed);
    void stop();
    void reverse();

private:
    int _pin1;
    int _pin2;
    bool _isReversed;
};
