#pragma once

#include "constants.hpp"
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
    void runServoPinTest();
    void writeServoAngle(uint8_t servoIdx, uint8_t angle);
    void writeRelayState(bool on);

    // Left front back then right front back
    Motor motors[4];
    uint8_t servoPins[SERVO_COUNT];
    uint8_t servoAngles[SERVO_COUNT];
    bool relayState;

    uint8_t serialBuff[COMMAND_SIZE];
    uint8_t serialBuffInd;
    unsigned long lastPacketTime;
    unsigned long lastMotorPacketTime;
    bool motorFailsafeEngaged;
    int servoTestIndex;
    int previousServoTestIndex;
    int servoTestAngle;
    int servoTestDir;
    unsigned long lastServoTestStepTime;
    unsigned long servoBootHoldStart;
    bool servoBootHoldActive;
};
