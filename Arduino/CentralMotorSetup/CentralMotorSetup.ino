// SEND EXAMPLE OF SERIAL CAN MODULE
// unsigned char send(unsigned long id, uchar ext, uchar rtrBit, uchar len, const uchar *buf);
// SUPPORT: joney.sui@longan-labs.cc
#include <Serial_CAN_Module.h>
#include <SoftwareSerial.h>

Serial_CAN can;

#define can_tx  2           // tx of serial can module connect to D2
#define can_rx  3           // rx of serial can module connect to D3

unsigned char start_ctrl[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
unsigned char velocity[6] = {0x00, 0x00, 0x00, 0x02, 0x00, 0x00};
unsigned char exit_ctrl[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

void setup()
{
    Serial.begin(9600);
    can.begin(can_tx, can_rx, 9600);      // tx, rx
    Serial.println("begin");
    if(can.canRate(CAN_RATE_1000))
    {
        Serial.println("set can rate ok");
    }
    else
    {
        Serial.println("set can rate fail");
    }
    can.send(0x00, 0, 0, 8, start_ctrl);
    delay(100);
    can.send(0x00, 0, 0, 6, velocity);
    delay(100);
    can.send(0x00, 0, 0, 8, exit_ctrl);
}

unsigned long id = 0;
unsigned char dta[8];

// send(unsigned long id, byte ext, byte rtrBit, byte len, const byte *buf);
void loop()
{
  if(can.recv(&id, dta))
    {
    Serial.print("GET DATA FROM ID: ");
    Serial.println(id);
    for(int i=0; i<8; i++)
    {
        Serial.print("0x");
        Serial.print(dta[i], HEX);
        Serial.print('\t');
    }
    Serial.println();
    }
}

// END FILE
