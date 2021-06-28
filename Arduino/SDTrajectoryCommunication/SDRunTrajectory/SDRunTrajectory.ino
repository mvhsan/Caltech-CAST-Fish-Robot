/*
   Tyler Nguyen 2020
   tylernguyen@caltech.edu
   Reads a fin trajectory stored on an SD Card, and commands servos.
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
boolean firstTime = true;

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

//Data to send (outCmd) and that are received (inCmd) to/from SBGC
//SerialCommand inCmd, outCmd;

//SBGC_cmd_angles_data_t curAngleData;


void setup() {
  //Serial initialization for USB
  Serial.begin(115200);

/*
  //Serial and cmd initialization for SBGC communicatiom
  Serial1.begin(115200);
  SBGC_Demo_setup(&Serial1);
  outCmd.init(SBGC_CMD_GET_ANGLES);
  sbgc_parser.send_cmd(outCmd, 0);
*/

  //Wait for user input to begin
  Serial.println("Press ENTER to begin: ");
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
  //Run the trajectory 'timesToRun' number of times
  if (timesToRun > 0) {

    //If running the trajectory starting with the vector, we must go to the beginning of the trajectory
    if (firstTime) {
      //Seek to beginning of SD file
      sdFile.seek(0);

      //Reset timestep
      timestep = 0;

      //Mark the starting time on the Arduino for the trajectory to keep pace with
      startTime = millis();

      firstTime = false;
    }

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
    }

    //If it is the correct time to run the vector, send it to the Arduino
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

      //Mark that the next vector is ready to be received from SD
      newdata = false;
      parsed = false;
    }

    if (!sdFile.available()) {
      timesToRun--;
      firstTime = true;
      return;
    }

  } else {
    //No longer running trajectory. Close SD file and stop program
    sdFile.close();
    Serial.println("Done!");
    Serial.end();
    while (1);
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
