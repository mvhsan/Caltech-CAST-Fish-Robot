/*
 * Maven Holst
 * 07/26/21
 * 
 * Reads motor encoder readings periodically using asynchronous serial communication
 */

#include <SoftwareSerial.h>

#define rx 19
#define tx 18
#define WORD_BITS (8 * sizeof(unsigned int))
#define SIZE 5

unsigned char REQUEST = '1';

//unsigned char* bitArray = (char*)calloc(5, sizeof(unsigned char));

unsigned char bitArray[5];

static inline void setIndex(size_t idx) {
    bitArray[idx / WORD_BITS] |= (1 << (idx % WORD_BITS));
}

static inline int getIndex(size_t idx) {
    return ( bitArray[idx / WORD_BITS] & (1 << (idx % WORD_BITS)) );
}

float getAngle() {
    int encoderPosition = 0;
    for (unsigned int i = 10; i < 24; i++) {
        encoderPosition += (getIndex(i) << (i - 10));
    }
    return ((float)encoderPosition / 16384.0f) * 360.0f; // degrees
}

int getTurn() {
    int turnCount = 0;
    for (unsigned int i = 24; i < 40; i++) {
        turnCount += (getIndex(i) << (i - 24));
    }
    return turnCount;
}

void getReadings() {
    Serial1.write(REQUEST);
    //while ((char)(Serial1.read() & 0xFF) != REQUEST) {}
    while ((char)(Serial1.read()) != REQUEST) {}
    Serial1.readBytes(bitArray, 5);
}

void setup() {
    Serial1.begin(1000000);
    Serial.begin(115200);
    //while (!Serial.available()) {}
}

void loop() {
    getReadings();
    Serial.println(getAngle());
    Serial.println(getTurn());
    delay(100);
}
