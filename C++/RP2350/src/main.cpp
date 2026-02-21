#include <Arduino.h>

#include "Robot.hpp"

Robot &r = Robot::getRobot();

void setup()
{
    r.setup();
}

void loop()
{
    r.update();
}
