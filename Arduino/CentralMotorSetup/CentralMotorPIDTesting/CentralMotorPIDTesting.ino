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

float kP = 1e-6;
float kD = 0;
volatile float PIDError = 0;
volatile float PIDErrorD = 0;
volatile float prevTime = 0;
volatile float currentTime = 0; 
volatile float centralAngle = 0;
volatile float lastCentralAngle = 90;

// The target angle for the PID loop
float yawAngle = 90;

Servo C;
int centralMS;

void setup() {
    C.attach(motorPin);
    C.writeMicroseconds(1000);
    timerStart = 0;
    attachInterrupt(digitalPinToInterrupt(PWMPin), calcEncoderSignal, CHANGE);
    Serial.begin(115200);
}

void loop() {
    centralPID();
    Serial.println(getCentralAngle());
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

    currentTime = micros();
    PIDErrorD = (centralAngle - lastCentralAngle) / (currentTime - prevTime);

    centralMS = (round) (1000 + kP*PIDError + kD*PIDErrorD);
    C.writeMicroseconds(centralMS);

    lastCentralAngle = centralAngle;
    prevTime = currentTime;
}
