#include "Robot.hpp"
#include "constants.hpp"

static Robot *instance = NULL;
static const int SERVO_TEST_MIN_DEG = 70;
static const int SERVO_TEST_MAX_DEG = 130;
static const int SERVO_TEST_CENTER_DEG = (SERVO_TEST_MIN_DEG + SERVO_TEST_MAX_DEG) / 2;
static const int SERVO_TEST_STEP_DEG = 2;
static const unsigned long SERVO_TEST_STEP_MS = 25;
static const bool SERVO_TEST_ENABLED = false;
static const int SERVO_REST_DEG = SERVO_TEST_CENTER_DEG;
static const char *SERVO_TEST_LABELS[3] = {
    "D0 (ARM_SERVO)",
    "D5 (CLAW_TOUCH)",
    "D6 (CLAW)",
};

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
    servoTestIndex = 0;
    previousServoTestIndex = -1;
    servoTestAngle = SERVO_TEST_MIN_DEG;
    servoTestDir = 1; // start by moving forward
    lastServoTestStepTime = 0;
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

    arm.write(SERVO_REST_DEG);
    claw[0].write(SERVO_REST_DEG);
    claw[1].write(SERVO_REST_DEG);

    if (SERVO_TEST_ENABLED)
    {
        // Kick off the first pin test from the low bound moving forward.
        arm.write(servoTestAngle);
        Serial.println("[SERVO TEST] Cycling pins one at a time: D0 -> D5 -> D6");
        Serial.print("[SERVO TEST] Active pin: ");
        Serial.println(SERVO_TEST_LABELS[servoTestIndex]);
    }
    else
    {
        Serial.print("[SERVO] Holding rest position at ");
        Serial.println(SERVO_REST_DEG);
    }
}

void Robot::update()
{
    parseInput();
    if (SERVO_TEST_ENABLED)
    {
        runServoPinTest();
    }
}

void Robot::runServoPinTest()
{
    unsigned long now = millis();
    if (now - lastServoTestStepTime < SERVO_TEST_STEP_MS)
    {
        return;
    }
    lastServoTestStepTime = now;

    Servo *outputs[3] = {&arm, &claw[0], &claw[1]};

    // Only recenter non-active outputs when active pin changes.
    if (previousServoTestIndex != servoTestIndex)
    {
        for (int i = 0; i < 3; i++)
        {
            if (i != servoTestIndex)
            {
                outputs[i]->write(SERVO_TEST_CENTER_DEG);
            }
        }
        previousServoTestIndex = servoTestIndex;
    }

    servoTestAngle += servoTestDir * SERVO_TEST_STEP_DEG;
    if (servoTestAngle >= SERVO_TEST_MAX_DEG)
    {
        servoTestAngle = SERVO_TEST_MAX_DEG;
        servoTestDir = -1;
    }
    else if (servoTestAngle <= SERVO_TEST_MIN_DEG)
    {
        servoTestAngle = SERVO_TEST_MIN_DEG;
        if (servoTestDir < 0)
        {
            // Completed one full back-and-forth on this pin, move to next pin.
            servoTestDir = 1;
            servoTestIndex = (servoTestIndex + 1) % 3;
            Serial.print("[SERVO TEST] Active pin: ");
            Serial.println(SERVO_TEST_LABELS[servoTestIndex]);
        }
    }

    outputs[servoTestIndex]->write(servoTestAngle);
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
