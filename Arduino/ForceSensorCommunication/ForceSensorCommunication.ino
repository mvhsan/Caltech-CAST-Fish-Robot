
/*
 * James Chen
 * 07/16/21
 * 
 * Gets force measurements from force sensor and then sends them to MATLAB.
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>

char terminator = '\n'; //Char that marks end of individual message from MATLAB

int ledPin = LED_BUILTIN;

String debugMsg;

void setup() {
  pinMode(ledPin, OUTPUT);

  //setup from SDRunTrajectory

  //Serial initialization for USB
  Serial.begin(115200);
  Serial.setTimeout(100);

  //Wait for MATLAB input to begin
  while (Serial.available() == 0) {
    delay(200);
  }

  //MATLAB has outputted something, now clear Serial
  debugMsg = Serial.readStringUntil(terminator);
  Serial.println(debugMsg);

  digitalWrite(ledPin, HIGH);
}
void loop() {
  // put your main code here, to run repeatedly:

}
