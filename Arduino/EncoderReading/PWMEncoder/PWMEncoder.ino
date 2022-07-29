/*
 * Maven Holst
 * 07/26/21
 * 
 * Reads motor encoder readings periodically using PWM
 */

#include <SoftwareSerial.h>
#include <stdio.h>

const int PWMPin = 2;
//#define interruptPin 0

float PWMMax = 910.17;
float PWMMin = 0.0556;

volatile unsigned long timerStart;
volatile int lastInterruptTime;
volatile int pulseTime;

volatile float angle = 0;

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
    timerStart = 0;
    attachInterrupt(digitalPinToInterrupt(PWMPin), calcSignal, CHANGE);
    Serial.begin(115200);
    //while (!Serial.available()) {}
}

void loop() {
    // calcSignal();
    //angle = getAngle();
    //Serial.println(strcat("Angle (degrees): ", fprintf()));
    Serial.println(getAngle());
    delay(100);
}
