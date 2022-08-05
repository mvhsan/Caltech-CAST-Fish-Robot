/*
 * Maven Holst
 * 07/29/21
 * 
 * Sample P-D loop tuned for no fin load on motor. Capable of 
 * quickly finding the target angle within +/- 10 degrees or so.
 * Can only rotate motor in one direction, need to plan relay
 * logic to swap ESC connections for bidirectional control.
 */

#include <Servo.h>
#include <SoftwareSerial.h>
#include <stdio.h>

const int motorPin = 3;
const int PWMPin = 2;

float PWMMax = 910.17;
float PWMMin = 0.0556;

volatile unsigned long timerStart;
volatile int lastInterruptTime;
volatile int pulseTime;

float kP = 5; //2.15
float kD = 500;
volatile float PIDError = 0;
volatile float PIDErrorD = 0;
volatile float prevTime = 0;
volatile float currentTime = 0; 
volatile float centralAngle = 0;
volatile float lastCentralAngle = 90;

// The target angle for the PID loop
float yawAngle = 200;

Servo C;
int centralMS;

void setup() {
    C.attach(motorPin);
    C.writeMicroseconds(1000);
    delay(5000);
    timerStart = 0;
    attachInterrupt(digitalPinToInterrupt(PWMPin), calcEncoderSignal, CHANGE);
    Serial.begin(115200);
}

void loop() {
    centralPID();
    Serial.println(getCentralAngle());
    Serial.println(centralMS);
    Serial.println(PIDErrorD);
    delay(10);
}

void calcEncoderSignal() {
    lastInterruptTime = micros();
    if (digitalRead(PWMPin) == HIGH) {
        timerStart = micros();
    } else if (timerStart != 0) {
        pulseTime = ((volatile int)micros() - timerStart);
        timerStart = 0;
    }
}

float getCentralAngle() {
    return ((float)pulseTime - PWMMin) * (16383.0f / 16384.0f) * 360.0f / (PWMMax - PWMMin);
}

void centralPID() {
    centralAngle = getCentralAngle();
    PIDError = centralAngle - yawAngle;

    currentTime = (float) millis();
    PIDErrorD = (centralAngle - lastCentralAngle) / (currentTime - prevTime);

    centralMS = (round) (1000 + kP*PIDError + kD*PIDErrorD);
    if (centralMS > 2000) {
        centralMS = 2000;
    } else if (centralMS < 1000) {
        centralMS = 1000;
    }
    C.writeMicroseconds(centralMS);

    lastCentralAngle = centralAngle;
    prevTime = currentTime;
}
