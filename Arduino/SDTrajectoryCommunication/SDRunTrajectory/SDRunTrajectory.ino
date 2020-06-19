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


//initialization of the different angles. Will change
float pwmRotation = 90;
float upDownAngle = 90;
float leftRightAngle = 90;
float timestep = 0;


int timesToRun = 2; //Stores the number of times to run trajectory. 1-indexed.
boolean firstTime = true; //Flag whether this is the first time that trajectory has run (important for the first vector timing)

CONTROL control(5, 6, 9, 10, 11, 1);/*constructor of the object Control -> the 5 first numbers are for the number of
  the pins 5,6 for right,left, 9,10 for up, down and 11 for continuous servo*/


//Characters that define vector
boolean recvInProgress = false;
const char startMarker = '<';
const char delimiter = ',';
const char endMarker = '>';

//Stores the starting time in milliseconds of trajectory
long startTime = millis();

//Indices for storage of vector
byte curVar;
byte curChar;

//Current character received from SD card
char received;

//Flag whether vector has been read in full
boolean newdata = false;

//Flag whether vector has been parsed
boolean parsed = false;

void setup() {
  //Serial initialization
  Serial.begin(250000);
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
  sdFile = SD.open("test.txt", O_READ);

  //If file cannot be opened, stop program
  if (!sdFile) {
    Serial.println("error opening file");
    while (1);
  }

  control.init();   //Initialize servos


  Serial.print("Starting trajectory... Initialization took: ");
  Serial.print((millis() - startTime) / 1000);
  Serial.println("seconds.");
}



void loop() {

  while (timesToRun > 0){
    //Seek to beginning of file
    sdFile.seek(0);
    //Reset timestep
    timestep = 0;
    //Set the start time for trajectory to keep pace with
    startTime = millis();

    while (sdFile.available()) {
      //Receive new character from SD card
      if (!newdata) {
        receiveChar();
        continue;
      } else if (!parsed){
          //Parse data into floats
          timestep = atof(recvChars[0]);
          pwmRotation = atof(recvChars[1]);
          upDownAngle = atof(recvChars[2]);
          leftRightAngle = atof(recvChars[3]);
      } 
      if ((millis() - startTime) >= timestep * 1000) {
          //Command servos
          float upDownMS = map(upDownAngle, 0, 180, 1000, 2000);
          float leftRightMS = map(leftRightAngle, 0, 180, 1000, 2000);
          control.controlUpDown(upDownMS);
          control.controlLeftRight(leftRightMS);

          Serial.print(millis()-startTime);Serial.print(" ");Serial.print(timestep*1000, 8);Serial.print(" ");Serial.print(upDownAngle,8);Serial.print(" ");Serial.println(leftRightAngle,8);
          
          

          //Serial.print("step ");
          //Serial.println(timestep);
          //Serial.print("pace ");
          //Serial.println((millis() - startTime) / 1000.0);
          newdata = false;
        }
      }
      timesToRun--;
    }
  //No longer running trajectory. Close SD file
    sdFile.close();
    Serial.println("Done!");
    Serial.end();
  
}

//Retrieve a character from SD card and put it in recvChars
void receiveChar() {
  received = sdFile.read();
  if (recvInProgress == true) {
    if (received == endMarker) { //End of 3-value pair
      newdata = true;
      recvInProgress = false;
      return;
    } else if (received == delimiter) { //End of a singular value in vector
      recvChars[curVar][curChar] = '\0'; //Terminate string
      curVar++;
      curChar = 0;
      return;
    } else { //Add received character to the end of the current angle.
      recvChars[curVar][curChar] = received;
      curChar++;
    }
  }
  else if (received == startMarker) { //Start of 3-value vector
    recvInProgress = true;

    // Clear received characters
    recvChars[0][0] = '\0';
    recvChars[0][1] = '\0';
    recvChars[0][2] = '\0';
    recvChars[0][3] = '\0';

    curVar = 0;
    curChar = 0;

    newdata = false;
  }
}
