/*
   Tyler Nguyen 2020
   tylernguyen@caltech.edu
   Communicates with MATLAB over Serial in order to receive trajectory coordinates on Arduino.
   Run ArduinoCode first, then run MATLAB. Will run until it has received the entire trajectory.

   Data comes in the form of characters sent in the following format:
   "<yaw,pitch,roll>"
      
   Writes data to SD card
*/

#include <string.h>
#include <SPI.h>
#include <SD.h>

//Filename to open and store to. Will need confirmation before overwriting
String filename = "test.txt";

//Stores an opened file on the SD card
File sdFile;

//Flags whether data is still being received
boolean running = false;

//Flags whether a vector has been fully received from SD and is ready to be parsed
boolean newData = false;

//Stores a vector as it is being received, is cleared when data has been parsed
String receivedVector = "";

void setup() {
  //Begin serial communication
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Wait for serial port to connect
  while (!Serial) {
    ;
  }

  //Initialize SD board pins
  pinMode(53, OUTPUT);

  Serial.print("Initializing SD card... ");

  if (!SD.begin(10)) {
    Serial.println("Initialization failed!");
    while (1);
  }
  Serial.println("Initialization done.");

  //Wait until MATLAB is ready for transmitting
  while (Serial.available() <= 0) {
    delay(1000);
  }

  //If the file exists already, we'll wipe it clean
  if (SD.exists(filename)) {
    SD.remove("test.txt");
  }

  sdFile = SD.open(filename, FILE_WRITE);

  //If file cannot be opened, display error and stop the program
  if (!sdFile) {
    Serial.println("Error opening file. Program will not halt");
    while (1);
  }

  running = true;
  Serial.println("File opened successfully. Waiting for MATLAB data...");
  //Signal to MATLAB that Arduino is ready for data transfer
}

void loop() {
  
  if (running) {
    //Receive a character from serial if available
    recvData();
    //If a full vector has been received, store it on the SD card
    if (newData) {
      newData = false;
      writeVectorToSD();
      returnData();
    }
  }
}

void recvData() {
  char startMarker = '<';
  char delimiter = ',';
  char endMarker = '>';
  char doneMarker = '!';
  static boolean recvInProgress = false;

  char received; //Current character received over serial

  while (Serial.available() > 0 && newData == false) {
    received = Serial.read();

    if (received == doneMarker) { //Done transmitting
      running = false;
      sdFile.close();
      Serial.println("Halting..");
      return;
    }

    if (recvInProgress == true) {
      if (received == endMarker) { //End of 3-value pair
        newData = true;
        recvInProgress = false;
      }

      receivedVector += received;
    }
    else if (received == startMarker) { //Start of 3-value vector
      recvInProgress = true;

      // Clear received characters
      receivedVector = "<";
    }
  }
}

void returnData() {
  //send received vector back to MATLAB
  Serial.println(receivedVector);
}

void writeVectorToSD() {
  //Write to SD:
  sdFile.println(receivedVector);
}
