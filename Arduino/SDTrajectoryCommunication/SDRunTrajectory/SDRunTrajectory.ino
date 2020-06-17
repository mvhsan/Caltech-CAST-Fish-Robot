/*
   Tyler Nguyen 2020
   tylernguyen@caltech.edu
   Reads a trajectory stored on an SD Card, and commands servos.
*/

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <CONTROL.h> //import control library : library homemade that contains all the functions to control the different servos

File sdFile;

//initialization of data structures that will store received characters from SD card
const byte numChars = 16; //Max num chars to expect when reading from serial for each value
char recvChars[4][numChars]; //Stores the current vector values. (timestep,phi,theta,psi)


//initialization of the different angles
float pwmRotation = 90;
float upDownAngle = 90;
float leftRightAngle = 90;
float timestep = 0;

CONTROL control(5, 6, 9, 10, 11, 1);/*constructor of the object Control -> the 5 first numbers are for the number of
  the pins 5,6 for right,left, 9,10 for up, down and 11 for continuous servo*/

//Data received

void setup() {
  //Serial initialization
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println("Press ENTER to begin: ");
  while (Serial.available() == 0); // Wait for Serial connection to connect (USB)

  //Initialize SD card
  pinMode(53, OUTPUT);

  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  //Open file on SD for reading
  sdFile = SD.open("test.txt");

  //If file cannot be opened:
  if (!sdFile) {
    Serial.println("error opening file");
    while (1);
  }

  //Initialize servos
  control.init();

  Serial.println("Reading...");
  
  //Characters that define vector
  static boolean recvInProgress = false;
  char startMarker = '<';
  char delimiter = ',';
  char endMarker = '>';

  byte curVar;
  byte curChar;

  char received; //Current character received from SD card
  boolean newdata = false; //Flag whether vector has been read in full

  while (sdFile.available()){
    received = sdFile.read();

    if (recvInProgress == true){
      if (received == endMarker){ //End of 3-value pair
        newdata = true;
        recvInProgress = false;
        return;
      } else
      if (received == delimiter){ //End of a singular value in vector
        recvChars[curVar][curChar] = '\0'; //Terminate string
        curVar++;
        curChar = 0;
        continue;
      } else { //Add received character to the end of the current angle.
        recvChars[curVar][curChar] = received;
        curChar++;
      }
    }
    else if (received == startMarker){ //Start of 3-value vector
      recvInProgress = true;

    // Clear received characters
      recvChars[0][0] = '\0';
      recvChars[0][1] = '\0';
      recvChars[0][2] = '\0';
      recvChars[0][3] = '\0';

      curVar = 0;
      curChar = 0;
    }

    //If all of vector has been read, parse the numbers and perform the command with servos
    if (newdata){
      //Parse data into floats
      timestep = atof(recvChars[0]);
      pwmRotation = atof(recvChars[1]);
      upDownAngle = atof(recvChars[2]);
      leftRightAngle = atof(recvChars[3]);

      //Command servos
      float upDownMS = map(upDownAngle, 0, 180, 1000, 2000);
      float leftRightMS = map(leftRightAngle, 0, 180, 1000, 2000);
      control.controlUpDown(upDownMS);
      control.controlLeftRight(leftRightMS);
    }
  }
  Serial.println("Done!");

  sdFile.close();
}



void loop() {

}
