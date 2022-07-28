
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
int PWM_U = (round) (60.4 * 5.56 + 1500.0);
int PWM_D = (round) (-60.4 * 5.56 + 1500.0);
int PWM_mid = 1500;

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
      U.writeMicroseconds(PWM_mid);
      D.writeMicroseconds(PWM_mid);
      L.writeMicroseconds(PWM_U);
      R.writeMicroseconds(PWM_D);
    } else {
      Serial.println("Setting fin to midpoint");
      U.writeMicroseconds(PWM_mid);
      D.writeMicroseconds(PWM_mid);
      L.writeMicroseconds(PWM_D);
      R.writeMicroseconds(PWM_U);
    }
  }

  lastSwitchState = switchState;
}
