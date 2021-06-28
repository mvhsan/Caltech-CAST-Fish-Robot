/*
   James Chen 2021
   jameschen@caltech.edu
   Combines code for reading fin trajectory and commanding servos with code for receiving
   trajectory from MATLAB and writing data to SD card.
   
   Tyler Nguyen 2020
   tylernguyen@caltech.edu
   Reads a fin trajectory stored on an SD Card, and commands servos.

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
#include <Servo.h>
#include <inttypes.h>
//#include <SBGC_Arduino.h>
//#include <SBGC.h>


File sdFile;

//initialization of data structures that will store received characters from SD card
const byte numChars = 16; //Max num chars to expect when reading from serial for each value
char recvChars[6][numChars]; //Stores the servo angles received. (timestep,yaw,up,down,left,right)

//Servos
Servo U, D, L, R;

//initialization of the different angles. Will change
float timestep = 0;
float yawAngle = 90;
float upAngle = 90;
float downAngle = 90;
float leftAngle = 90;
float rightAngle = 90;

int upMS, downMS, leftMS, rightMS;

int lastUMicroseconds = 0;
int lastLMicroseconds = 0;

//Stores the number of times to run trajectory. 1-indexed.
int timesToRun = 1;

//Flag whether this is the first time that trajectory has run (important for the first vector timing)
boolean trajectoryReset = true;

//Characters that define vector
boolean recvInProgress = false;
const char startMarker = '<';
const char delimiter = ',';
const char endMarker = '>';

//Stores the starting time in milliseconds of trajectory
long startTime = millis();
long initTime = millis();

//Indices for storage of vector
byte curVar;
byte curChar;

//Current character received from SD card
char received;

//Flag whether full vector has been received
boolean newdata = false;

//Flag whether vector has been parsed
boolean parsed = false;

//Flag whether Arduino code is waiting for MATLAB instruction
boolean waitingForInstruction = true;

String MATLABmessage;

//Data to send (outCmd) and that are received (inCmd) to/from SBGC
//SerialCommand inCmd, outCmd;

//SBGC_cmd_angles_data_t curAngleData;


void setup() {

  //setup from SDRunTrajectory
  
  //Serial initialization for USB
  Serial.begin(115200);

  //Wait for user input to begin
  Serial.println("Write to serial to initialize SD");
  while (Serial.available() == 0);

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

  //Attach servos. Note: Pin 10 was acting strangely, and the servo was being sent false signals. Try to avoid using it in the future.
  R.attach(9);
  L.attach(8);
  U.attach(6);
  D.attach(5);

  Serial.print("Starting trajectory... Initialization took: ");
  Serial.print((millis() - startTime) / 1000);
  Serial.println("seconds.");

  initTime = millis();




  
}

void loop() {

  if (waitingForInstruction) {
    if (Serial.available >= 8) {        // complete message available in Serial
      MATLABmessage = Serial.read();

      if (MATLABmessage == "{instr1}") {  // update trajectory
        updatingSDTrajectory = true;
        waitingForInstruction = false;
        Serial.write("{received}");
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
      
        updatingSDTrajectory = true;
        Serial.println("File opened successfully. Waiting for MATLAB data...");
        //Signal to MATLAB that Arduino is ready for data transfer
      }
      else if (MATLABmessage == "{instr2}") { // perform trajectory
        updatingSDtrajectory = false;
        waitingForInstruction = false;
        Serial.write("{received}");
      }
    }
  }
  else {
    //If running the trajectory starting with the vector, we must go to the beginning of the trajectory
    if (trajectoryReset) {
      //Seek to beginning of SD file
      sdFile.seek(0);
  
      //Reset timestep
      timestep = 0;
  
      //Mark the starting time on the Arduino for the trajectory to keep pace with
      startTime = millis();
  
      trajectoryReset = false;
    }

    if (updatingSDTrajectory) {
      if (updatingSDTrajectory) {
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
    else {

      // check if there are vector points available to read
      if (sdFile.available()) {

        //If a full vector has not been read yet, receive more characters from SD card, otherwise parse the data
        if (!newdata) {
          receiveChar();
          return;
        } else if (!parsed) {
          //Parse the character data into floats if it hasn't been parsed yet
          timestep = atof(recvChars[0]);
          yawAngle = atof(recvChars[1]);
          upAngle = atof(recvChars[2]);
          downAngle = atof(recvChars[3]);
          leftAngle = atof(recvChars[4]);
          rightAngle = atof(recvChars[5]);
    
          //Convert the angle into milliseconds for the servo signal
          upMS = (round) (upAngle / 180.0 * 1000.0 + 1520.0);
          downMS = (round) (downAngle / 180.0 * 1000.0 + 1520.0);
          leftMS = (round) (leftAngle / 180.0 * 1000.0 + 1520.0);
          rightMS = (round) (rightAngle / 180.0 * 1000.0 + 1520.0);
          
          parsed = true;
        }
      
  
        //If it is the correct time to run the vector, send it to the servos
        if (((millis() - startTime) >= timestep * 1000)) {
          if (leftMS != lastLMicroseconds) {
            L.writeMicroseconds(leftMS);
            R.writeMicroseconds(rightMS);
            lastLMicroseconds = leftMS;
          }
          
          //If desired angle has changed, send it to servos. Otherwise, there's no need to send it again
          if (upMS != lastUMicroseconds) {
            U.writeMicroseconds(upMS);
            D.writeMicroseconds(downMS);
            lastUMicroseconds = upMS;
          }
          Serial.println();     //start off with a new line
          Serial.print(millis() - initTime);  Serial.print(" ");
          Serial.print(millis() - startTime); Serial.print(" ");
          Serial.print(timestep * 1000, 8);   Serial.print(" ");
          Serial.print(upAngle);   Serial.print(" ");
          Serial.print(downAngle); Serial.print(" ");;
          Serial.print(leftAngle); Serial.print(" ");
          Serial.print(rightAngle);
      
      //      Serial.print(SBGC_ANGLE_TO_DEGREE(curAngleData.sensor_data[0].imu_data)); Serial.print(" ");
      //      Serial.println(SBGC_ANGLE_TO_DEGREE(curAngleData.sensor_data[1].imu_data));
      
      
          //Mark that the next vector is ready to be received from SD
          newdata = false;
          parsed = false;
        }
      }
      else {
        //No longer running trajectory. Close SD file and stop program
        waitingForInstruction = true;
        Serial.println("Done!");
        
        
      }
    }
  }
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
    recvChars[0][4] = '\0';
    recvChars[0][5] = '\0';

    curVar = 0;
    curChar = 0;

    newdata = false;
  }
}









/*

*/

#include <string.h>
#include <SPI.h>
#include <SD.h>

//Filename to open and store to. Will need confirmation before overwriting
String filename = "test.txt";

//Stores an opened file on the SD card
File sdFile;

//Flags whether data is still being received
boolean updatingSDTrajectory = false;

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

  /*  Overwrite protection code
    //Open file on SD for writing. If file already exists on SD, ask whether to overwrite
    if (SD.exists(filename)) {
      Serial.print(filename);
      Serial.println(" already exists. Overwrite? (Y/N):");
      Serial.println(">>>");



      //If user inputs Y, go ahead and overwrite, but otherwise stop the program
      if (Serial.read() == 'Y'){
        SD.remove("test.txt");
        Serial.println("File overwritten.");
      } else {
        Serial.println("File will not be overwritten. Program will now halt.");
        while (1);
      }
    }

      sdFile = SD.open(filename, FILE_WRITE);
  */

  
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
      updatingSDTrajectory = false;
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
