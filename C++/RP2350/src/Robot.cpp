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
}

void Robot::setup()
{
    Serial.begin(115200);

    for (int i = 0; i < 4; i++)
        motors[i].setup();

    motors[0].reverse();
    motors[1].reverse();
}

void Robot::update()
{
    parseInput();
}

void Robot::parseInput()
{
    if (Serial.available())
    {
        uint8_t b = Serial.read();

        if (serialBuffInd != 0 || b == START_BYTE)
        {
            // Write into buffer
            serialBuff[serialBuffInd++] = b;

            if (serialBuffInd == 5)
            {
                // Verify not corrupt
                uint8_t crc = serialBuff[0] ^ serialBuff[1] ^ serialBuff[2] ^ serialBuff[3];

                if (crc == serialBuff[4])
                    handlePacket();

                serialBuffInd = 0;
            }
        }
    }
}

void Robot::handlePacket()
{
    switch (serialBuff[1])
    {
    case SET_MOTOR_SPEED:
        motors[serialBuff[2]].setSpeed(serialBuff[3]);
        break;
    }
}
