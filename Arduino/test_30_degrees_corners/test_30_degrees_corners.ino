
/*
 * Maven Holst
 * 07/26/21
 * 
 * Sets roll and pitch to +/- 30 degrees, in 5 second intervals, to test tail range of motion
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

//Servos
Servo U, D, L, R;
double midpoint = 1400.0;
int PWM_high = (round) (47.1 * 5.56 + midpoint);
int PWM_low = (round) (-47.1 * 5.56 + midpoint);
int PWM_mid = (round) (midpoint);
int PWM_high_oct = (round) (30.4 * 5.56 + midpoint);
int PWM_low_oct = (round) (-30.4 * 5.56 + midpoint);

void setup() {
  //attach servos to pins
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  U.writeMicroseconds(PWM_high);
  D.writeMicroseconds(PWM_low);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);

  delay(500);

  U.writeMicroseconds(PWM_high_oct);
  D.writeMicroseconds(PWM_low_oct);
  L.writeMicroseconds(PWM_high_oct);
  R.writeMicroseconds(PWM_low_oct);

  delay(500);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_high);
  R.writeMicroseconds(PWM_low);

  delay(500);

  U.writeMicroseconds(PWM_low_oct);
  D.writeMicroseconds(PWM_high_oct);
  L.writeMicroseconds(PWM_high_oct);
  R.writeMicroseconds(PWM_low_oct);

  delay(500);

  U.writeMicroseconds(PWM_low);
  D.writeMicroseconds(PWM_high);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);

  delay(500);

  U.writeMicroseconds(PWM_low_oct);
  D.writeMicroseconds(PWM_high_oct);
  L.writeMicroseconds(PWM_low_oct);
  R.writeMicroseconds(PWM_high_oct);

  delay(500);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_low);
  R.writeMicroseconds(PWM_high);

  delay(500);

  U.writeMicroseconds(PWM_high_oct);
  D.writeMicroseconds(PWM_low_oct);
  L.writeMicroseconds(PWM_low_oct);
  R.writeMicroseconds(PWM_high_oct);

  delay(500);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);
}

void loop() {

}
