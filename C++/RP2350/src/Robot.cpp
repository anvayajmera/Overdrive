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
static const unsigned long MOTOR_FAILSAFE_MS = 250;
static const uint32_t SERVO_PWM_FREQ_HZ = 50;
static const uint8_t SERVO_PWM_RES_BITS = 16;
static const uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RES_BITS) - 1UL;
static const uint32_t SERVO_PERIOD_US = 20000;
static const uint32_t SERVO_MIN_US = 500;
static const uint32_t SERVO_MAX_US = 2500;
static const unsigned long SERVO_BOOT_HOLD_MS = 8000;
static const unsigned long SERVO_BOOT_PREP_MS = 50;
static const char *SERVO_TEST_LABELS[SERVO_COUNT] = {
    "GPIO16 (ARM_SERVO)",
    "GPIO26 (CLAW_LEFT_SERVO)",
    "GPIO32 (CLAW_RIGHT_SERVO)",
    "GPIO5 (PLATFORM_BACK_SERVO)",
};

static uint32_t angleToServoDuty(uint8_t angle)
{
    uint8_t bounded = (uint8_t)constrain((int)angle, 0, 180);
    uint32_t pulseUs = map(bounded, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    return (uint32_t)(((uint64_t)pulseUs * SERVO_PWM_MAX_DUTY) / SERVO_PERIOD_US);
}

Robot &Robot::getRobot()
{
    if (!instance)
        instance = new Robot();

    return *instance;
}

Robot::Robot()
{
    motors[0] = Motor(
        MOTOR_LEFT_FRONT_IN1,
        MOTOR_LEFT_FRONT_IN2);
    motors[1] = Motor(
        MOTOR_LEFT_REAR_IN1,
        MOTOR_LEFT_REAR_IN2);
    motors[2] = Motor(
        MOTOR_RIGHT_FRONT_IN1,
        MOTOR_RIGHT_FRONT_IN2);
    motors[3] = Motor(
        MOTOR_RIGHT_REAR_IN1,
        MOTOR_RIGHT_REAR_IN2);

    servoPins[0] = ARM_SERVO_PIN;
    servoPins[1] = CLAW_LEFT_SERVO_PIN;
    servoPins[2] = CLAW_RIGHT_SERVO_PIN;
    servoPins[3] = PLATFORM_BACK_SERVO_PIN;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        servoAngles[i] = SERVO_REST_DEG;
    }
    relayState = false;

    serialBuffInd = 0;
    lastPacketTime = millis();
    lastMotorPacketTime = millis();
    motorFailsafeEngaged = false;
    servoTestIndex = 0;
    previousServoTestIndex = -1;
    servoTestAngle = SERVO_TEST_MIN_DEG;
    servoTestDir = 1; // start by moving forward
    lastServoTestStepTime = 0;
    servoBootHoldStart = 0;
    servoBootHoldActive = false;
}

void Robot::setup()
{
    Serial.begin(921600);

    for (int i = 0; i < 4; i++)
        motors[i].setup();

    motors[0].reverse();
    motors[1].reverse();

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        pinMode(servoPins[i], OUTPUT);
        digitalWrite(servoPins[i], LOW);
    }
    delay(SERVO_BOOT_PREP_MS);

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        writeServoAngle(i, SERVO_REST_DEG);
    }
    servoBootHoldStart = millis();
    servoBootHoldActive = true;

    pinMode(RELAY_PIN, OUTPUT);
    writeRelayState(false);

    if (SERVO_TEST_ENABLED)
    {
        // Kick off the first pin test from the low bound moving forward.
        writeServoAngle(servoTestIndex, servoTestAngle);
        Serial.println("[SERVO TEST] Cycling pins one at a time: GPIO16 -> GPIO21 -> GPIO32 -> GPIO5");
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

    unsigned long now = millis();
    // if (!motorFailsafeEngaged && (now - lastMotorPacketTime > MOTOR_FAILSAFE_MS))
    // {
    //     for (int i = 0; i < 4; i++)
    //     {
    //         motors[i].stop();
    //     }
    //     motorFailsafeEngaged = true;
    // }

    if (SERVO_TEST_ENABLED)
    {
        runServoPinTest();
    }

    if (servoBootHoldActive)
    {
        unsigned long now = millis();
        if (now - servoBootHoldStart < SERVO_BOOT_HOLD_MS)
        {
            for (int i = 0; i < SERVO_COUNT; i++)
            {
                writeServoAngle(i, SERVO_REST_DEG);
            }
        }
        else
        {
            servoBootHoldActive = false;
        }
    }
}

void Robot::writeServoAngle(uint8_t servoIdx, uint8_t angle)
{
    if (servoIdx >= SERVO_COUNT)
    {
        return;
    }

    uint8_t boundedAngle = (uint8_t)constrain((int)angle, 0, 180);
    servoAngles[servoIdx] = boundedAngle;
    analogWrite(servoPins[servoIdx], angleToServoDuty(boundedAngle));
}

void Robot::writeRelayState(bool on)
{
    relayState = on;
    bool pinHigh = RELAY_ACTIVE_LOW ? !on : on;
    digitalWrite(RELAY_PIN, pinHigh ? HIGH : LOW);
}

void Robot::runServoPinTest()
{
    unsigned long now = millis();
    if (now - lastServoTestStepTime < SERVO_TEST_STEP_MS)
    {
        return;
    }
    lastServoTestStepTime = now;

    // Only recenter non-active outputs when active pin changes.
    if (previousServoTestIndex != servoTestIndex)
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (i != servoTestIndex)
            {
                writeServoAngle(i, SERVO_TEST_CENTER_DEG);
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
            servoTestIndex = (servoTestIndex + 1) % SERVO_COUNT;
            Serial.print("[SERVO TEST] Active pin: ");
            Serial.println(SERVO_TEST_LABELS[servoTestIndex]);
        }
    }

    writeServoAngle((uint8_t)servoTestIndex, (uint8_t)servoTestAngle);
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
    uint8_t arg1 = serialBuff[2];
    uint8_t arg2 = serialBuff[3];
    uint8_t arg3 = serialBuff[4];

    switch (cmd)
    {
    case SET_MOTOR_SPEED:
        if (arg1 < 4)
        {
            int speed = (arg3 > 1) ? -(int)arg2 : (int)arg2;
            motors[arg1].setSpeed(speed);
            lastMotorPacketTime = millis();
            motorFailsafeEngaged = false;
        }
        break;
    case SET_SERVO_ANGLE:
        if (arg1 < SERVO_COUNT)
        {
            writeServoAngle(arg1, arg2);
            servoBootHoldActive = false;
        }
        break;
    case SET_RELAY_STATE:
        writeRelayState(arg1 != 0);
        break;
    default:
        break;
    }
}
