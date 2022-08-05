/*
 * Maven Holst
 * 07/29/21
 * 
 * Reads motor encoder readings periodically using PWM. Can tell
 * motor to rotate shaft with switch.
 */

#include <SoftwareSerial.h>
#include <stdio.h>
#include <Servo.h>

const int PWMPin = 2;
const int motorPin = 3;
const int switchPin = 7;
const int ledPin = 13;

float PWMMax = 910.17;
float PWMMin = 0.0556;

volatile unsigned long timerStart;
volatile int lastInterruptTime;
volatile int pulseTime;

volatile float angle = 0;

Servo C;
int PWMLow = 1000;
int PWMTest = 1050;
int switchState = 0;
int lastSwitchState = 0;

void calcSignal() {
    lastInterruptTime = micros();
    if (digitalRead(PWMPin) == HIGH) {
        timerStart = micros();
    } else if (timerStart != 0) {
            pulseTime = ((volatile int)micros() - timerStart);
            timerStart = 0;
    }
}

float getAngle() {
    return ((float)pulseTime - PWMMin) * (16383.0f / 16384.0f) * 360.0f / (PWMMax - PWMMin);
}

void setup() {
    pinMode(switchPin, INPUT);
    pinMode(ledPin, OUTPUT);
    
    timerStart = 0;
    attachInterrupt(digitalPinToInterrupt(PWMPin), calcSignal, CHANGE);
    Serial.begin(115200);
    //while (!Serial.available()) {}

    C.attach(motorPin);
    C.writeMicroseconds(PWMLow);
}

void loop() {
    switchState = digitalRead(switchPin);

    if (switchState != lastSwitchState) {
        if (switchState == HIGH) {
            Serial.println("Setting central motor throttle value to test value");
            digitalWrite(ledPin, HIGH);
            C.writeMicroseconds(PWMTest);
        } else {
            Serial.println("Setting central motor throttle value to low");
            digitalWrite(ledPin, LOW);
            C.writeMicroseconds(PWMLow);
        }
    }

    lastSwitchState = switchState;
    
    Serial.println(getAngle());
    delay(100);
}
