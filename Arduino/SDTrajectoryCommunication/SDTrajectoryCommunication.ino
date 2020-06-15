/*
 * Tyler Nguyen 2020
 * tylernguyen@caltech.edu
 * Communicates with MATLAB over Serial in order to receive trajectory coordinates on Arduino.
 * Run ArduinoCode first, then run MATLAB. Will run until it has received the entire trajectory.
 * Writes to EEPROM
 */

#include <string.h>
#include <SPI.h>
#include <SD.h>

File sdFile;

boolean running = true;
boolean newData = false;

String receivedVector = "";

void setup() {    
  //Begin serial communication
  Serial.begin(9600);
  Serial.setTimeout(100);

  // Wait for serial port to connect
  while(!Serial){;}

  //Initialize SD
  pinMode(53, OUTPUT);
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  
  //Open file on SD for writing
  SD.remove("test.txt");
  sdFile = SD.open("test.txt", FILE_WRITE);

  //If file cannot be opened:
   if (!sdFile){
    Serial.println("error opening file");
    while (1);
   }

}

void loop() {
  if(running){
    recvData();
    if (newData){
      newData = false;
      writeVectorToSD();
      returnData();
    }
  }
}

void recvData(){
  char startMarker = '<';
  char delimiter = ',';
  char endMarker = '>';
  char doneMarker = '!';
  static boolean recvInProgress = false;

  char received; //Current character received over serial

  while (Serial.available() > 0 && newData == false){
    received = Serial.read();

      if (received == doneMarker){ //Done transmitting
        running = false;
        sdFile.close();
        Serial.println("Halting..");
        return;
      }

    if (recvInProgress == true){
      if (received == endMarker){ //End of 3-value pair
        newData = true;
        recvInProgress = false;
      }

      receivedVector += received;
    }
    else if (received == startMarker){ //Start of 3-value vector
      recvInProgress = true;

    // Clear received characters
      receivedVector = "<";
    }
  }
}

void returnData(){
    //send received vector back to MATLAB
    Serial.println(receivedVector);
}

void writeVectorToSD(){
    //Write to SD:
    
  sdFile.println(receivedVector);
}
