#include "motor.hpp"

Motor::Motor(int IN1, int IN2) {
    _pin1 = IN1;
    _pin2 = IN2;
    _isReversed = false;
}

void Motor::setup() {
    pinMode(_pin1, OUTPUT);
    pinMode(_pin2, OUTPUT);
    stop();
}

void Motor::stop() {
    analogWrite(_pin1, 0);
    analogWrite(_pin2, 0);
    digitalWrite(_pin1, LOW);
    digitalWrite(_pin2, LOW);
}

void Motor::reverse() {
    _isReversed = !_isReversed;
}

void Motor::setSpeed(int speed) {
    int val = map(constrain(speed, 0, 100), 0, 100, 0, 255);
    if (!_isReversed) {
        digitalWrite(_pin2, LOW);
        analogWrite(_pin1, val);
    } else {
        digitalWrite(_pin1, LOW);
        analogWrite(_pin2, val);
    }
}