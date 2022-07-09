#include <Servo.h>

const int motorPin = 3;
const int switchPin = 8;

Servo C;
int PWMLow = 1000;
int PWMHigh = 2000;
int switchState = 0;
int lastSwitchState = 0;

void setup() {
    pinMode(switchPin, INPUT);

    C.attach(motorPin);

    Serial.begin(115200);
    //while(!Serial.available()) {}
    
    Serial.println("Testing central motor");
}

void loop() {
    switchState = digitalRead(switchPin);

    if (switchState != lastSwitchState) {
        if (switchState == HIGH) {
            Serial.println("Setting central motor throttle value to high");
            C.writeMicroseconds(PWMHigh);
        } else {
            Serial.println("Setting central motor throttle value to low");
            C.writeMicroseconds(PWMLow);
        }
    }

    lastSwitchState = switchState;
}