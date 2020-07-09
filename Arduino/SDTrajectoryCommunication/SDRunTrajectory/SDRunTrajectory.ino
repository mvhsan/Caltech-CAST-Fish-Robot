/*
   Tyler Nguyen 2020
   tylernguyen@caltech.edu
   Reads a trajectory stored on an SD Card, and commands servos.
*/

#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <inttypes.h>
#include <SBGC_Arduino.h>
#include <SBGC.h>


File sdFile;

//initialization of data structures that will store received characters from SD card
const byte numChars = 16; //Max num chars to expect when reading from serial for each value
char recvChars[4][numChars]; //Stores the current vector values. (timestep,phi,theta,psi)

//Servos
Servo L;
Servo R;
Servo U;
Servo D;

//initialization of the different angles. Will change
float pwmRotation = 90;
float upDownAngle = 90;
float leftRightAngle = 90;
float timestep = 0;

int lastUDMicroseconds = 0;
int lastLRMicroseconds = 0;

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

//Indices for storage of vector
byte curVar;
byte curChar;

//Current character received from SD card
char received;

//Flag whether full vector has been received
boolean newdata = false;

//Flag whether vector has been parsed
boolean parsed = false;

SerialCommand inCmd;
SerialCommand outCmd;

SBGC_cmd_realtime_data_t curRTData;


void setup() {
  //Serial initialization
  Serial.begin(250000);
  Serial.setTimeout(100);

  Serial1.begin(115200);
  SBGC_Demo_setup(&Serial1);
  outCmd.init(SBGC_CMD_REALTIME_DATA_3);
  sbgc_parser.send_cmd(outCmd, 0);


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
}



void loop() {
  //Run the trajectory 'timesToRun' number of times
  while (timesToRun > 0) {
    //Seek to beginning of SD file
    sdFile.seek(0);

    //Reset timestep
    timestep = 0;

    //Mark the starting time on the Arduino for the trajectory to keep pace with
    startTime = millis();

    while (sdFile.available()) {

      //If a full vector has not been read yet, receive more characters from SD card, otherwise parse the data
      if (!newdata) {
        receiveChar();
        continue;
      } else if (!parsed) {
        //Parse the character data into floats if it hasn't been parsed yet
        timestep = atof(recvChars[0]);
        pwmRotation = atof(recvChars[1]);
        upDownAngle = atof(recvChars[2]);
        leftRightAngle = atof(recvChars[3]);
      }

      //If it is the correct time to run the vector, send it to the Arduino
      if ((millis() - startTime) >= timestep * 1000) {
        //Convert the angle into MS for the servo signal
        int upDownMS = (round) (upDownAngle / 180 * 1000 + 1000);
        int leftRightMS = (round) (leftRightAngle / 180 * 1000 + 1000);

        //If desired angle has changed, send it to servos. Otherwise, there's no need to send it again
        if (upDownMS != lastUDMicroseconds) {
          U.writeMicroseconds(upDownMS);
          D.writeMicroseconds(3000 - upDownMS);
          lastUDMicroseconds = upDownMS;
        }

        if (leftRightMS != lastLRMicroseconds) {
          L.writeMicroseconds(leftRightMS);
          R.writeMicroseconds(2940 - leftRightMS); //
          lastLRMicroseconds = leftRightMS;
        }





        Serial.print(millis() - startTime); Serial.print(" "); Serial.print(timestep * 1000, 8);
        Serial.print(" "); Serial.print(upDownMS); Serial.print(" "); Serial.print(3000 - upDownMS);
        Serial.print(" "); Serial.print(leftRightMS); Serial.print(" "); Serial.println(2940 - leftRightMS);

        sbgc_parser.send_cmd(outCmd, 0);


        if (sbgc_parser.read_cmd()) {
          //Receive incoming data
          inCmd = sbgc_parser.in_cmd;
          //Unpacks the incoming data into curRTData
          SBGC_cmd_realtime_data_unpack(curRTData, inCmd);
        }

        Serial.println(curRTData.imu_angle[0]);

        //Mark that the next vector is ready to be received from SD
        newdata = false;
      }
    }
    timesToRun--;
  }

  //No longer running trajectory. Close SD file and stop program
  sdFile.close();
  Serial.println("Done!");
  Serial.end();
  while (1);

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
