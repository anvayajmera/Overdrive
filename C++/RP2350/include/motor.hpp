#include <Arduino.h>

struct Motor
{
public:
    // IN1 is pwm, IN2 is dir. Call setup to function.
    Motor(int IN1, int IN2);

    // Sets the pinModes of 1 and 2 to be output.
    void setup();

    // Sets the speed of the motors (0-100). Note: retains same direction, will retain reversed direction.
    void setSpeed(int speed);

    // Shortcut for setSpeed(0);
    void stop();

    // Reverses the motors
    void reverse();

private:
    int IN1;
    int IN2;
    int speed;
};