#include <Arduino.h>

#include "Robot.hpp"

// D0 = Big Servo at front
// D5 = Tiny servo with touch sensor
// D6 = other tiny servo

Robot &r = Robot::getRobot();

void setup()
{
    r.setup();
}

void loop()
{
    r.update();
}
