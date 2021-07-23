/*
 *    James Chen | July 2021
 *    jameschen@caltech.edu
 *    
 *    This is the Arduino code for the Arduino that controls the force sensor and the multi-channel relay
 *    connected to the servos. It connects to a computer via serial and sends force measurements upon being
 *    pinged. It also implements a safety stop for the servos, as if the force exceeds some threshold, it
 *    will shut off power to the servos using the multi-channel relay it controls. The servos will remain
 *    unpowered until the system is entirely reset.
 */

float reading;
long t=0;
int timeStep;
int nPoints;
int counter=0;

int servoRelay1 = 13;
int servoRelay2 = 12;
int servoRelay3 = 11;
int servoRelay4 = 10;

int FORCE_THRESHOLD = 600;

const int ledPin =  LED_BUILTIN;  //Arduinos have a built in LED, the pin number is a preset constant

String MATLABmessage;
String debugMsg;
char terminator = '\n'; //Char that marks end of individual message from MATLABmessage

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  //Serial initialization for USB
  Serial.begin(115200);
  Serial.setTimeout(100);

  //Wait for MATLAB input to begin
  while (Serial.available() == 0) {
    delay(100);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  //MATLAB has outputted something, now clear Serial
  debugMsg = Serial.readStringUntil(terminator);
  Serial.println(debugMsg);
  Serial.println("serial start");

  pinMode(servoRelay1, OUTPUT);       //initialize relay pins to be outputs
  pinMode(servoRelay2, OUTPUT);
  pinMode(servoRelay3, OUTPUT);
  pinMode(servoRelay4, OUTPUT);
  digitalWrite(servoRelay1, LOW);     //turn on all servo relays
  digitalWrite(servoRelay2, LOW);
  digitalWrite(servoRelay3, LOW);
  digitalWrite(servoRelay4, LOW);
}

void loop() {
  delay(100);
  reading = analogRead(0);
  Serial.println(reading);
//  if(Serial.available() != 0){        //wait for complete message to arrive in serial
//    MATLABmessage = Serial.readStringUntil(terminator);   //read until newline char to get to end of message
//    Serial.println(MATLABmessage);
//    digitalWrite(servoRelay1, !digitalRead(servoRelay1));
//    if (MATLABmessage.equals("get force")) {
//      reading = analogRead(0);
//      Serial.println(reading);
//      if (reading >= FORCE_THRESHOLD) {
//        digitalWrite(servoRelay1, HIGH);
//        digitalWrite(servoRelay2, HIGH);
//        digitalWrite(servoRelay3, HIGH);
//        digitalWrite(servoRelay4, HIGH);
//        Serial.println("FORCE ERROR");
//        while (1) {
//          digitalWrite(LED_BUILTIN, HIGH);  //force was too high, shut off servos until system is reset
//          delay(100);
//          digitalWrite(LED_BUILTIN, LOW);
//          delay(100);
//        }
//      }
//
//    }
//  }
}
