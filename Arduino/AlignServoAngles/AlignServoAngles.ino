#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

Servo U, D, L, R;

int PWM = 1500;

void setup() {
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  U.writeMicroseconds(PWM);
  D.writeMicroseconds(PWM);
  L.writeMicroseconds(PWM);
  R.writeMicroseconds(PWM);
}

void loop() {

}
