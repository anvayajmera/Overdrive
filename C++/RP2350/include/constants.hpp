#pragma once

#include <Arduino.h>

// ESP32 DevKit Type-C pin map.
// Each motor uses two PWM-capable pins (IN1/IN2) for bidirectional drive.
static const int MOTOR_LEFT_FRONT_IN1 = 17;
static const int MOTOR_LEFT_FRONT_IN2 = 21;
static const int MOTOR_LEFT_REAR_IN1 = 25;
static const int MOTOR_LEFT_REAR_IN2 = 19;
static const int MOTOR_RIGHT_FRONT_IN1 = 27;
static const int MOTOR_RIGHT_FRONT_IN2 = 14;
static const int MOTOR_RIGHT_REAR_IN1 = 13;
static const int MOTOR_RIGHT_REAR_IN2 = 18;



static const uint32_t MOTOR_PWM_FREQ_HZ = 20000;
static const uint8_t MOTOR_PWM_RES_BITS = 8;
static const uint8_t MOTOR_PWM_MAX_DUTY = 255;
// Treat very small commands as zero to prevent idle PWM hum.
static const uint8_t MOTOR_SPEED_DEADBAND = 0;
// Detach PWM outputs when stopped to fully silence the driver.
static const bool MOTOR_DETACH_AT_STOP = false;

// Servos (all PWM-capable pins).
static const int ARM_SERVO_PIN = 16;
static const int CLAW_LEFT_SERVO_PIN = 26;
static const int CLAW_RIGHT_SERVO_PIN = 32;
static const int PLATFORM_BACK_SERVO_PIN = 5;
static const uint8_t SERVO_COUNT = 4;

// Relay output.
static const int RELAY_PIN = 33;
static const bool RELAY_ACTIVE_LOW = true;

// Serial protocol
static const uint8_t START_BYTE = 0xAA;
static const uint8_t SET_MOTOR_SPEED = 0x01;
static const uint8_t SET_SERVO_ANGLE = 0x02;
static const uint8_t SET_RELAY_STATE = 0x03;
static const uint8_t COMMAND_SIZE = 5;
