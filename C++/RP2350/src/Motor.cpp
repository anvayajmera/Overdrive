#include "Motor.hpp"
#include "constants.hpp"

Motor::Motor()
{
    _pin1 = -1;
    _pin2 = -1;
    _isReversed = false;
    _attached = false;
}

Motor::Motor(int IN1, int IN2)
{
    _pin1 = IN2;
    _pin2 = IN1;
    _isReversed = false;
    _attached = false;
}

void Motor::setup()
{
    attachIfNeeded();
    stop();
}

void Motor::stop()
{
    analogWrite(_pin1, 0);
    analogWrite(_pin2, 0);
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
    if (MOTOR_DETACH_AT_STOP && _attached)
    {
        pinMode(_pin1, INPUT);
        pinMode(_pin2, INPUT);
        _attached = false;
    }
}

void Motor::reverse()
{
    _isReversed = !_isReversed;
}

void Motor::setSpeed(int speed)
{
    if (abs(speed) <= MOTOR_SPEED_DEADBAND)
    {
        stop();
        return;
    }

    attachIfNeeded();

    int val = map(constrain(abs(speed), 0, 100), 0, 100, 0, MOTOR_PWM_MAX_DUTY);

    speed *= _isReversed ? -1 : 1;

    if (speed > 0)
    {
        analogWrite(_pin2, 0);
        analogWrite(_pin1, val);
    }
    else
    {
        analogWrite(_pin1, 0);
        analogWrite(_pin2, val);
    }
}

void Motor::attachIfNeeded()
{
    if (_attached)
    {
        return;
    }

    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    _attached = true;
}
