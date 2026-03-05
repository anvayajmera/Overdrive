#include "Robot.hpp"
#include "constants.hpp"

static Robot *instance = NULL;

Robot &Robot::getRobot()
{
    if (!instance)
        instance = new Robot();

    return *instance;
}

Robot::Robot()
{
    motors[0] = Motor(MOTOR_LEFT_FRONT);
    motors[1] = Motor(MOTOR_LEFT_REAR);
    motors[2] = Motor(MOTOR_RIGHT_FRONT);
    motors[3] = Motor(MOTOR_RIGHT_REAR);

    serialBuffInd = 0;
    lastPacketTime = millis();
}

void Robot::setup()
{
    Serial.begin(921600);

    for (int i = 0; i < 4; i++)
        motors[i].setup();

    motors[0].reverse();
    motors[1].reverse();

    arm.attach(ARM_SERVO, 500, 2500);
    claw[0].attach(CLAW_TOUCH, 500, 2500);
    claw[1].attach(CLAW, 500, 2500);

    arm.write(120);
    for (int i = 0; i < 2; i++)
    {
        claw[i].write(90);
    }
}

void Robot::update()
{
    parseInput();
}

void Robot::parseInput()
{
    while (Serial.available())
    {
        uint8_t b = Serial.read();

        if (b == START_BYTE)
        {
            serialBuffInd = 0;
        }

        if (serialBuffInd != 0 || b == START_BYTE)
        {
            if (serialBuffInd < COMMAND_SIZE)
            {
                serialBuff[serialBuffInd++] = b;
            }

            if (serialBuffInd == COMMAND_SIZE)
            {
                lastPacketTime = millis();
                yield(); // Vital to make code work for some reason
                handlePacket();
                serialBuffInd = 0;
            }
        }
    }
}

void Robot::handlePacket()
{
    uint8_t cmd = serialBuff[1];
    uint8_t motorIdx = serialBuff[2];
    uint8_t val = serialBuff[3];
    uint8_t sign = serialBuff[4];

    switch (cmd)
    {
    case SET_MOTOR_SPEED:
        if (motorIdx < 4)
        {
            int speed = (sign > 1) ? -(int)val : (int)val;
            motors[motorIdx].setSpeed(speed);
        }
        break;
    default:
        break;
    }
}
