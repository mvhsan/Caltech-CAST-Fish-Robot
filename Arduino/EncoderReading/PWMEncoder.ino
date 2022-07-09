#include <SoftwareSerial.h>

#define PWMPin 2
#define interruptPin 0

float PWMMax = 910.17;
float PWMMin = 0.0556;

volatile unsigned long timerStart;
volatile int lastInterruptTime;
volatile int pulseTime;

void calcSignal() {
    last_interrupt_time = micros();
    if (digitalRead(PWMPin) == HIGH) {
        timerStart = micros;
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
    attachInterrupt(interruptPin, calcSignal, CHANGE);
    Serial.begin(115200);
    while (!Serial.available()) {}
}

void loop() {
    // calcSignal();
    Serial.println("Angle (degrees): " + getAngle());
    delay(100);
}