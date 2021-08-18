#include <SPI.h>
#include <mcp2515.h>
 
struct can_frame canMsg;
struct can_frame canMsg1;
struct can_frame canMsg2;
struct can_frame canMsg3;

MCP2515 mcp2515(10);
int cntr = 0;
unsigned long oldTime = 0;

int readValue;
 
void setup() {
  canMsg1.can_id  = 0x01;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xFF;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xFF;
  canMsg1.data[3] = 0xFF;
  canMsg1.data[4] = 0xFF;
  canMsg1.data[5] = 0xFF;
  canMsg1.data[6] = 0xFF;
  canMsg1.data[7] = 0xFC;

  canMsg2.can_id  = 0x01;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0xFF;
  canMsg2.data[1] = 0xFF;
  canMsg2.data[2] = 0xFF;
  canMsg2.data[3] = 0xFF;
  canMsg2.data[4] = 0xFF;
  canMsg2.data[5] = 0xFF;
  canMsg2.data[6] = 0xFF;
  canMsg2.data[7] = 0xFD;

  canMsg3.can_id  = 0x01;
  canMsg3.can_dlc = 8;
  canMsg3.data[0] = 0x7F;
  canMsg3.data[1] = 0xFF;
  canMsg3.data[2] = 0x85;
  canMsg3.data[3] = 0xA0;
  canMsg3.data[4] = 0x00;
  canMsg3.data[5] = 0x66;
  canMsg3.data[6] = 0x67;
  canMsg3.data[7] = 0xFF;
  Serial.begin(115200);
 
  mcp2515.reset();
  mcp2515.setNormalMode();
  mcp2515.setBitrate(CAN_1000KBPS);


  mcp2515.sendMessage(&canMsg1);
  delay(100);
  
  Serial.println("sent message, entered motor control mode");
  mcp2515.sendMessage(&canMsg3);

  delay(100);

  for (int i = 0; i < 8; i++) {
    Serial.println("index " + String(i) + ":   " + String(canMsg1.data[i]));
  }
}
 
void loop() {
  readValue = mcp2515.readMessage(&canMsg);
  Serial.println(readValue);
  if (readValue == MCP2515::ERROR_OK) {
    cntr++;
    Serial.println();
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }
  }
 
  if ((millis()-oldTime)>1000) {
    oldTime = millis();
    Serial.print(cntr);
    Serial.println(" msg/sec");
    cntr = 0;
  }
  Serial.print("enter motor mode: "); Serial.println(mcp2515.sendMessage(&canMsg1));
  delay(100);
  Serial.print("spin command: "); Serial.println(mcp2515.sendMessage(&canMsg3));
  delay(100);
}
