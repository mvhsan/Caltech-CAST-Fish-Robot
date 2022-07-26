
/*
 * Maven Holst
 * 07/26/21
 * 
 * Sets roll to 30 degrees to test tail range of motion
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

//Servos
Servo U, D, L, R;
int PWM_U = (round) (60.4 * 8.85 + 1500.0);
int PWM_D = (round) (-60.4 * 8.85 + 1500.0);
int PWM_R = 1500;
int PWM_L = 1500;

void setup() {
  //attach servos to pins
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  U.writeMicroseconds(PWM_U);
  D.writeMicroseconds(PWM_D);
  L.writeMicroseconds(PWM_L);
  R.writeMicroseconds(PWM_R);
}

void loop() {

}
