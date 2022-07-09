#include <Servo.h>

#define motorPin 3

Servo C;
int PWM = 1500;

void setup() {
    C.attach(motorPin);

    Serial.begin(115200);
    while(!Serial.available()) {}
    
    Serial.println("Testing central motor");

    C.writeMicroseconds(PWM);
}

void loop() {

}