#include <Arduino.h>

#include "Motor.hpp"
#include "constants.hpp"

// Run motors + relay + servos together continuously (no auto-stop).
static const unsigned long MOTOR_STEP_MS = 200;
static const unsigned long RELAY_STEP_MS = 600;

static const int SERVO_MIN_DEG = 70;
static const int SERVO_MAX_DEG = 130;
static const int SERVO_CENTER_DEG = (SERVO_MIN_DEG + SERVO_MAX_DEG) / 2;
static const unsigned long SERVO_STEP_MS = 120;

static const uint32_t SERVO_PWM_FREQ_HZ = 50;
static const uint8_t SERVO_PWM_RES_BITS = 16;
static const uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RES_BITS) - 1UL;
static const uint32_t SERVO_PERIOD_US = 20000;
static const uint32_t SERVO_MIN_US = 500;
static const uint32_t SERVO_MAX_US = 2500;

static Motor motors[4];
static int servoPins[SERVO_COUNT];
static int servoAngles[SERVO_COUNT];

static unsigned long startMs = 0;
static unsigned long lastMotorStepMs = 0;
static unsigned long lastRelayStepMs = 0;
static unsigned long lastServoStepMs = 0;
static bool relayOn = false;
static uint8_t servoIndex = 0;
static int servoDir = 1;

static uint32_t angleToDuty(int angle)
{
    int bounded = constrain(angle, 0, 180);
    uint32_t pulseUs = map(bounded, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    return (uint32_t)(((uint64_t)pulseUs * SERVO_PWM_MAX_DUTY) / SERVO_PERIOD_US);
}

static void writeServoAngle(uint8_t idx, int angle)
{
    servoAngles[idx] = angle;
    analogWrite(servoPins[idx], angleToDuty(angle));
}

static void stopAllMotors()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motors[i].setSpeed(0);
    }
}

static void setRelay(bool on)
{
    const bool level = RELAY_ACTIVE_LOW ? !on : on;
    digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

void setup()
{
    Serial.begin(921600);
    delay(150);

    motors[0] = Motor(MOTOR_LEFT_FRONT_IN1, MOTOR_LEFT_FRONT_IN2);
    motors[1] = Motor(MOTOR_LEFT_REAR_IN1, MOTOR_LEFT_REAR_IN2);
    motors[2] = Motor(MOTOR_RIGHT_FRONT_IN1, MOTOR_RIGHT_FRONT_IN2);
    motors[3] = Motor(MOTOR_RIGHT_REAR_IN1, MOTOR_RIGHT_REAR_IN2);

    for (uint8_t i = 0; i < 4; i++)
    {
        motors[i].setup();
    }
    motors[0].reverse();
    motors[1].reverse();

    servoPins[0] = ARM_SERVO_PIN;
    servoPins[1] = CLAW_LEFT_SERVO_PIN;
    servoPins[2] = CLAW_RIGHT_SERVO_PIN;
    servoPins[3] = PLATFORM_BACK_SERVO_PIN;

    for (uint8_t i = 0; i < SERVO_COUNT; i++)
    {
#if defined(ESP32)
        analogWriteFrequency(servoPins[i], SERVO_PWM_FREQ_HZ);
        analogWriteResolution(servoPins[i], SERVO_PWM_RES_BITS);
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
        analogWriteFreq(SERVO_PWM_FREQ_HZ);
        analogWriteRange((1 << SERVO_PWM_RES_BITS) - 1);
#endif
        writeServoAngle(i, SERVO_CENTER_DEG);
    }

    pinMode(RELAY_PIN, OUTPUT);
    setRelay(false);

    randomSeed(micros());
    startMs = millis();
    lastMotorStepMs = startMs;
    lastRelayStepMs = startMs;
    lastServoStepMs = startMs;

    Serial.println("[NOISE TEST] Running motors + relay + servos together.");
}

void loop()
{
    unsigned long now = millis();
    unsigned long elapsed = now - startMs;

    (void)elapsed;

    if (now - lastMotorStepMs >= MOTOR_STEP_MS)
    {
        lastMotorStepMs = now;
        for (uint8_t i = 0; i < 4; i++)
        {
            int speed = (int)random(-100, 101);
            if (abs(speed) < 20)
            {
                speed = 0;
            }
            motors[i].setSpeed(speed);
        }
    }

    if (now - lastRelayStepMs >= RELAY_STEP_MS)
    {
        lastRelayStepMs = now;
        relayOn = !relayOn;
        setRelay(relayOn);
    }

    if (now - lastServoStepMs >= SERVO_STEP_MS)
    {
        lastServoStepMs = now;
        int angle = servoAngles[servoIndex] + (servoDir * 4);
        if (angle >= SERVO_MAX_DEG)
        {
            angle = SERVO_MAX_DEG;
            servoDir = -1;
        }
        else if (angle <= SERVO_MIN_DEG)
        {
            angle = SERVO_MIN_DEG;
            servoDir = 1;
            servoIndex = (servoIndex + 1) % SERVO_COUNT;
        }
        writeServoAngle(servoIndex, angle);
    }
}
