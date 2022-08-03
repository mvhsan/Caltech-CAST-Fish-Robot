#include <Servo.h>

const int motorPin = 3;
const int switchPin = 7;
const int ledPin = 13;

Servo U, D, L, R, C;
int PWMLow = 1000;
int PWMTest = 1500;
int switchState = 0;
int lastSwitchState = 0;

double midpoint = 1400.0;
int PWM_U = (round) (47.1 * 5.56 + midpoint);
int PWM_D = (round) (-47.1 * 5.56 + midpoint);
int PWM_mid = (round) (midpoint);

float lastTime;
bool centered = true;

void setup() {
    pinMode(switchPin, INPUT);
    pinMode(ledPin, OUTPUT);


    R.attach(9);
    L.attach(8);
    U.attach(6);
    D.attach(5);
    
    C.attach(motorPin);
    C.writeMicroseconds(PWMLow);

    Serial.begin(115200);
    //while(!Serial.available()) {}
    
    Serial.println("Testing central motor");

    U.writeMicroseconds(PWM_mid);
    D.writeMicroseconds(PWM_mid);
    L.writeMicroseconds(PWM_mid);
    R.writeMicroseconds(PWM_mid);

    lastTime = millis();
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

    if ( (millis() - lastTime) > 5000) {
        lastTime = millis();
        if (centered) {
            centered = false;
            U.writeMicroseconds(PWM_U);
            D.writeMicroseconds(PWM_D);
            L.writeMicroseconds(PWM_mid);
            R.writeMicroseconds(PWM_mid);
        } else {
            centered = true;
            U.writeMicroseconds(PWM_mid);
            D.writeMicroseconds(PWM_mid);
            L.writeMicroseconds(PWM_mid);
            R.writeMicroseconds(PWM_mid);
        }
    }
}
