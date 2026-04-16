#include <Arduino.h>

// Motor pair test using the exact GPIO mapping you provided.
// Runs one motor at a time: forward -> stop -> backward -> stop.

struct MotorPair
{
    uint8_t in1;
    uint8_t in2;
};

static const MotorPair MOTORS[] = {
    // motor1: front left (IN1=GPIO17, IN2=GPIO21)
    {17, 21},
    // motor2: front right (IN1=GPIO27, IN2=GPIO14)
    {27, 14},
    // motor3: back right (IN1=GPIO13, IN2=GPIO18)
    {13, 18},
    // motor4: back left (IN1=GPIO25, IN2=GPIO19)
    {25, 19},
};

static const uint8_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);

static const uint8_t RELAY_PIN = 33;
static const bool RELAY_ACTIVE_LOW = true;

static const unsigned long STEP_ON_MS = 2000;
static const unsigned long STEP_OFF_MS = 1000;
static const unsigned long RELAY_ON_MS = 1000;
static const unsigned long RELAY_OFF_MS = 1000;

static void stopMotor(uint8_t in1, uint8_t in2)
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

static void forward(uint8_t in1, uint8_t in2)
{
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
}

static void backward(uint8_t in1, uint8_t in2)
{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

static void setRelay(bool on)
{
    // Toggle relay to verify wiring. If your relay is active-high, set RELAY_ACTIVE_LOW = false.
    const bool level = RELAY_ACTIVE_LOW ? !on : on;
    digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

void setup()
{
    Serial.begin(921600);
    delay(200);

    pinMode(RELAY_PIN, OUTPUT);
    setRelay(false);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        pinMode(MOTORS[i].in1, OUTPUT);
        pinMode(MOTORS[i].in2, OUTPUT);
        stopMotor(MOTORS[i].in1, MOTORS[i].in2);
    }

    Serial.println("[PAIR TEST] Ready.");
}

void loop()
{
    Serial.println("[PAIR TEST] Relay ON");
    setRelay(true);
    delay(RELAY_ON_MS);
    Serial.println("[PAIR TEST] Relay OFF");
    setRelay(false);
    delay(RELAY_OFF_MS);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        Serial.print("[PAIR TEST] Motor ");
        Serial.print(i + 1);
        Serial.print(" IN1=GPIO");
        Serial.print(MOTORS[i].in1);
        Serial.print(" IN2=GPIO");
        Serial.println(MOTORS[i].in2);

        Serial.println("  forward");
        forward(MOTORS[i].in1, MOTORS[i].in2);
        delay(STEP_ON_MS);

        Serial.println("  stop");
        stopMotor(MOTORS[i].in1, MOTORS[i].in2);
        delay(STEP_OFF_MS);

        Serial.println("  backward");
        backward(MOTORS[i].in1, MOTORS[i].in2);
        delay(STEP_ON_MS);

        Serial.println("  stop");
        stopMotor(MOTORS[i].in1, MOTORS[i].in2);
        delay(STEP_OFF_MS);
    }
}
