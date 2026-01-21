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
            if (serialBuffInd == COMMAND_SIZE)
            {
                yield(); // Vital to make code work for some reason

                // Serial.println(serialBuff[0]);
                // for (int i = 1; i < COMMAND_SIZE-1; i++) {
                //     crc = crc ^ serialBuff[i];
                //     Serial.println(serialBuff[i]);
                // }

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
        int speed = serialBuff[4] > 1 ? (int)serialBuff[3] * -1 : serialBuff[3];
        Serial.printf("Received set motor command: Motor %d set to speed %d\n", serialBuff[2], speed);

        motors[serialBuff[2]].setSpeed(speed);
        break;
    }
}
