
/*
 * Maven Holst
 * 07/26/21
 * 
 * Switches fin between midpoint and 30 degrees
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

//Switch pin and states
const int switchPin = 7;
int switchState = 0;
int lastSwitchState = 0;

//Servos
Servo U, D, L, R;
double midpoint = 1400.0;
int PWM_U = (round) (47.1 * 5.56 + midpoint);
int PWM_D = (round) (-47.1 * 5.56 + midpoint);
int PWM_mid = (round) (midpoint);

void setup() {
  //setup switch input
  pinMode(switchPin, INPUT);

  //attach servos to pins
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  //start at midpoint
  U.writeMicroseconds(PWM_mid);
  D.writeMicroseconds(PWM_mid);
  L.writeMicroseconds(PWM_mid);
  R.writeMicroseconds(PWM_mid);

  //start serial connection
  Serial.begin(115200);
  Serial.println("Testing fin range of motion");
}

void loop() {
  switchState = digitalRead(switchPin);

  if (switchState != lastSwitchState) {
    if (switchState == HIGH) {
      Serial.println("Setting fin to 30 degrees");
      U.writeMicroseconds(PWM_D);
      D.writeMicroseconds(PWM_U);
      L.writeMicroseconds(PWM_mid);
      R.writeMicroseconds(PWM_mid);
    } else {
      Serial.println("Setting fin to midpoint");
      U.writeMicroseconds(PWM_mid);
      D.writeMicroseconds(PWM_mid);
      L.writeMicroseconds(PWM_mid);
      R.writeMicroseconds(PWM_mid);
    }
  }

  lastSwitchState = switchState;
}
