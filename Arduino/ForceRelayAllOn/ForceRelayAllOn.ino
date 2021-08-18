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

}
