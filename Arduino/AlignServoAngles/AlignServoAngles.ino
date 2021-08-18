#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

Servo U, D, L, R;

int PWM = 1500;

void setup() {
  R.attach(22);
  L.attach(20);
  U.attach(18);
  D.attach(16);

  U.writeMicroseconds(PWM);
  D.writeMicroseconds(PWM);
  L.writeMicroseconds(PWM);
  R.writeMicroseconds(PWM);
}

void loop() {

}
