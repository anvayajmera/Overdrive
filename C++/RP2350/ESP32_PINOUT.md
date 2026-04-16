# ESP32 DevKit Type-C Pinout (Overdrive)

## Motors (8 PWM-capable pins)

| Motor | IN1 (PWM) | IN2 (PWM / reverse) |
| --- | --- | --- |
| Left Front | GPIO17 | GPIO21 |
| Left Rear | GPIO25 | GPIO19 |
| Right Front | GPIO27 | GPIO14 |
| Right Rear | GPIO13 | GPIO18 |

## Servos (4 PWM-capable pins)

| Servo | Pin |
| --- | --- |
| Arm Servo | GPIO16 |
| Claw Left Servo | GPIO26 |
| Claw Right Servo | GPIO32 |
| Platform Back Servo | GPIO5 |

## Relay

| Device | Pin | Notes |
| --- | --- | --- |
| Relay Control | GPIO33 | Active-low logic in firmware by default (`RELAY_ACTIVE_LOW = true`) |

Note:
- `GPIO5` is a boot strapping pin. It is used here for the platform servo to keep motor wiring short.
- If your board has occasional boot/upload issues, move only that servo signal from `GPIO5` to `GPIO22` and update constants.

## Wheel Order (Confirmed)

During the relay click test cycle, the motors run in this order:
1. Front left
2. Front right
3. Back right
4. Back left

This matches the motor pin mapping above (left front, right front, right rear, left rear).

## Serial Protocol (Jetson <-> ESP32)

- `0x01` `SET_MOTOR_SPEED`: `arg1=motor_id`, `arg2=speed(0-100)`, `arg3=1/2 (forward/reverse sign)`
- `0x02` `SET_SERVO_ANGLE`: `arg1=servo_id(0-3)`, `arg2=angle(0-180)`
- `0x03` `SET_RELAY_STATE`: `arg1=0/1 (OFF/ON logical state)`
