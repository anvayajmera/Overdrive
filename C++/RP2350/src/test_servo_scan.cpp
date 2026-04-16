#include <Arduino.h>



// Servo pin scan: tries a list of candidate GPIOs one by one.
// Watch the claw servo and note which GPIO makes it move.

static const uint8_t SERVO_SCAN_PINS[] = {
    22,
    23,
    26,
    4,
    15,
    12,
    2,
    0,
};
static const uint8_t SERVO_SCAN_COUNT =
    sizeof(SERVO_SCAN_PINS) / sizeof(SERVO_SCAN_PINS[0]);

static const uint32_t SERVO_PWM_FREQ_HZ = 50;
static const uint8_t SERVO_PWM_RES_BITS = 16;
static const uint32_t SERVO_PWM_MAX_DUTY = (1UL << SERVO_PWM_RES_BITS) - 1UL;
static const uint32_t SERVO_PERIOD_US = 20000;
static const uint32_t SERVO_MIN_US = 500;
static const uint32_t SERVO_MAX_US = 2500;

static const int SERVO_MIN_DEG = 70;
static const int SERVO_MAX_DEG = 130;
static const int SERVO_CENTER_DEG = (SERVO_MIN_DEG + SERVO_MAX_DEG) / 2;
static const unsigned long HOLD_MS = 900;
static const unsigned long PAUSE_MS = 600;

static const uint8_t MOTOR_IN_PINS[] = {
    17, 21, // front left
    27, 14, // front right
    13, 18, // back right
    25, 19  // back left
};
static const uint8_t MOTOR_PIN_COUNT =
    sizeof(MOTOR_IN_PINS) / sizeof(MOTOR_IN_PINS[0]);

static void stopAllMotors()
{
    for (uint8_t i = 0; i < MOTOR_PIN_COUNT; i++)
    {
        pinMode(MOTOR_IN_PINS[i], OUTPUT);
        digitalWrite(MOTOR_IN_PINS[i], LOW);
    }
}

static uint32_t angleToDuty(int angle)
{
    int bounded = constrain(angle, 0, 180);
    uint32_t pulseUs = map(bounded, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
    return (uint32_t)(((uint64_t)pulseUs * SERVO_PWM_MAX_DUTY) / SERVO_PERIOD_US);
}

void setup()
{
    Serial.begin(921600);
    delay(200);

    stopAllMotors();

    Serial.println("[SERVO SCAN] Starting servo GPIO scan.");
    Serial.println("[SERVO SCAN] Watching for claw-left movement.");
    Serial.print("[SERVO SCAN] Angles: ");
    Serial.print(SERVO_MIN_DEG);
    Serial.print(" -> ");
    Serial.print(SERVO_MAX_DEG);
    Serial.println();
    Serial.println("[SERVO SCAN] Press ENTER to test each pin.");
}

static void waitForEnter()
{
    while (Serial.available() > 0)
    {
        Serial.read();
    }
    while (Serial.available() == 0)
    {
        delay(10);
    }
    while (Serial.available() > 0)
    {
        Serial.read();
    }
}

void loop()
{
    for (uint8_t i = 0; i < SERVO_SCAN_COUNT; i++)
    {
        const uint8_t pin = SERVO_SCAN_PINS[i];
        Serial.print("[SERVO SCAN] Testing GPIO");
        Serial.println(pin);
        Serial.println("  Press ENTER to run sweep.");
        waitForEnter();

#if defined(ESP32)
        analogWriteFrequency(pin, SERVO_PWM_FREQ_HZ);
        analogWriteResolution(pin, SERVO_PWM_RES_BITS);
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
        analogWriteFreq(SERVO_PWM_FREQ_HZ);
        analogWriteRange((1 << SERVO_PWM_RES_BITS) - 1);
#endif

        analogWrite(pin, angleToDuty(SERVO_CENTER_DEG));
        delay(PAUSE_MS);
        analogWrite(pin, angleToDuty(SERVO_MIN_DEG));
        delay(HOLD_MS);
        analogWrite(pin, angleToDuty(SERVO_MAX_DEG));
        delay(HOLD_MS);
        analogWrite(pin, angleToDuty(SERVO_CENTER_DEG));
        delay(PAUSE_MS);

        pinMode(pin, INPUT);
        delay(300);

        Serial.println("  Sweep done. Press ENTER for next pin.");
        waitForEnter();
    }
}
