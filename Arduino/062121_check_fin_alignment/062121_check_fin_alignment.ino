
/*
 * James Chen
 * 06/21/21
 * 
 * Sets servos to common angle to adjust for discrepancies in alignment.
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

//Servos
Servo U, D, L, R;
int PWM = 1500;

int servoRelay1 = 13;
int servoRelay2 = 12;
int servoRelay3 = 11;
int servoRelay4 = 10;

void setup() {
  //attach servos to pins
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  digitalWrite(servoRelay1, LOW);
  digitalWrite(servoRelay2, LOW);
  digitalWrite(servoRelay3, LOW);
  digitalWrite(servoRelay4, LOW);

}

void loop() {
  //set servo angles, all share common angle
  R.writeMicroseconds(PWM);
  L.writeMicroseconds(PWM);
  U.writeMicroseconds(PWM);
  D.writeMicroseconds(PWM);

}
