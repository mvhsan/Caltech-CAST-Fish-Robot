
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
int PWM_high = (round) (60.4 * 5.56 + 1500.0);
int PWM_low = (round) (-60.4 * 5.56 + 1500.0);
int PWM_mid = 1500;

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

  delay(5000);

  U.writeMicroseconds(PWM_low);
  D.writeMicroseconds(PWM_high);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);

  delay(5000);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_high);
  R.writeMicroseconds(PWM_low);

  delay(5000);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_low);
  R.writeMicroseconds(PWM_high);

  delay(5000);

  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);
}

void loop() {

}
