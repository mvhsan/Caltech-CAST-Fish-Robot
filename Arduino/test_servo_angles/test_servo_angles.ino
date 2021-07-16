#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

Servo myServo;




void setup() {
  myServo.attach(6);

}

void loop() {
  myServo.writeMicroseconds(2150);

}
