#pragma once

#include "Motor.hpp"

class Robot
{
public:
    static Robot &getRobot();

    Robot();
    void setup();
    void parseInput();
    void update();

private:
    void handlePacket();

    // Left front back then right front back
    Motor motors[4];

    uint8_t serialBuff[5];
    uint8_t serialBuffInd;
};