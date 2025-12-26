#include <Arduino.h>
#include <motor.hpp>

Motor m(0, 1);

void setup()
{
  // put your setup code here, to run once:
  m.setup();
}

void loop()
{
  // put your main code here, to run repeatedly:
  m.setSpeed(60);
}
