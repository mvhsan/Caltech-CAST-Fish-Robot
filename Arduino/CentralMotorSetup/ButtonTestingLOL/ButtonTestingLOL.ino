#include <Servo.h>

const int ledPin = 13;
const int switchPin = 8;

Servo C;
int PWMLow = 1000;
int PWMHigh = 2000;
int switchState = 0;
int lastSwitchState = 0;

void setup() {
    pinMode(switchPin, INPUT);
    pinMode(ledPin, OUTPUT);

    Serial.begin(115200);
    //while(!Serial.available()) {}
    
    Serial.println("Testing central motor");
}

void loop() {
    switchState = digitalRead(switchPin);

    if (switchState != lastSwitchState) {
        if (switchState == HIGH) {
            Serial.println("Throttle high, turning LED on");
            digitalWrite(ledPin, HIGH);
        } else {
            Serial.println("Throttle low, turning LED off");
            digitalWrite(ledPin, LOW);
        }
    }
    
    lastSwitchState = switchState;
}
