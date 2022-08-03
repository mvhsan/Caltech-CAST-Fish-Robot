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

//Servos and Central Motor ESC
Servo U, D, L, R, C;

//initialization of the different angles. Will change
float timestep = 0;
float yawAngle = 90;
float upAngle = 90;
float downAngle = 90;
float leftAngle = 90;
float rightAngle = 90;

int upMS, downMS, leftMS, rightMS, centralMS;

int lastUMicroseconds = 0;
int lastLMicroseconds = 0;

//Pin for reading encoder PWM readings
int PWMPin = 2;

//Variables to calculate central encoder reading from PWM pulse time
float PWMMax = 910.17;
float PWMMin = 0.0556;

volatile unsigned long timerStart;
volatile int lastInterruptTime;
volatile int pulseTime;

//Variables for central motor PID loop (currently just P-D loop) - TODO: tune PID constants
float kP = 0;
float kD = 0;
volatile float PIDError = 0;
volatile float PIDErrorD = 0;
volatile float prevTime = 0;
volatile float currentTime = 0; 
volatile float centralAngle = 0;
volatile float lastCentralAngle = 90;

//Stores the number of times to run trajectory. 1-indexed.
int timesToRun = 1;

//Flag whether this is the first time that trajectory has run (important for the first vector timing)
boolean trajectoryReset = true;

//Flag whether trajectory on SD card is to be reset
boolean updatingSDTrajectory = false;

//Filename to open and store to. Will need confirmation before overwriting
String filename = "test.txt";

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
boolean newData = false;

//Flag whether vector has been parsed
boolean parsed = false;

//Flag whether Arduino code is waiting for MATLAB instruction
boolean waitingForInstruction = true;

String MATLABmessage;
String debugMsg;
char terminator = '\n'; //Char that marks end of individual message from MATLAB

//Stores a vector as it is being received, is cleared when data has been parsed
String receivedVector = "";

// constants won't change. Used here to set a pin number:
const int ledPin =  LED_BUILTIN;// the number of the LED pin

void setup() {
  pinMode(ledPin, OUTPUT);

  //setup from SDRunTrajectory

  //Serial initialization for USB
  Serial.begin(115200);
  Serial.setTimeout(100);

  //Wait for MATLAB input to begin
  while (Serial.available() == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  //MATLAB has outputted something, now clear Serial
  debugMsg = Serial.readStringUntil(terminator);
  Serial.println(debugMsg);

  //Initialize SD card
  pinMode(53, OUTPUT);

  if (!SD.begin(10)) {
    Serial.println("SD-fail");
    while (1) {
    }
  }

  //If file cannot be opened, stop program
  sdFile = SD.open("test.txt", FILE_WRITE);
  if (!sdFile) {
    Serial.println("SD-fail");
    while (1) {
    }
  }

  Serial.println("SD-success");
  
  initTime = millis();
  timerStart = 0;
  attachInterrupt(digitalPinToInterrupt(PWMPin), calcEncoderSignal, CHANGE);
}

void loop() {

  if (waitingForInstruction) {
    //Attach servos. Note: Pin 10 was acting strangely, and the servo was being sent false signals. Try to avoid using it in the future.
    R.detach();
    L.detach();
    U.detach();
    D.detach();
    if (Serial.available() >= 6) {        // complete message available in Serial
      MATLABmessage = Serial.readStringUntil(terminator);
      Serial.println(MATLABmessage);
      if (MATLABmessage.equals("instr1")) {  // update trajectory
        digitalWrite(ledPin, HIGH);
        updatingSDTrajectory = true;
        waitingForInstruction = false;
        delay(.5);
        Serial.println("received");
      
        //If the target SD file exists already, we'll wipe it clean
        if (SD.exists("test.txt")) {
          SD.remove("test.txt");
        }

        sdFile = SD.open("test.txt", FILE_WRITE);

//        //If file cannot be opened, display error and stop the program
//        if (!sdFile) {
//          Serial.println("SD-fail\n");
//          while (1);
//        }

        //Serial.println("File opened successfully. Waiting for MATLAB data...");
        //Signal to MATLAB that Arduino is ready for data transfer
      }
      
      else if (MATLABmessage.equals("instr2")) {  // perform trajectory
        waitingForInstruction = false;
        updatingSDTrajectory = false;
        trajectoryReset = true;                   // servos and SD card need to be initialized to perform trajectory
        delay(.5);
        Serial.println("received");
      }
    }
  }

  // now performing instruction instead of receiving it
  else {

    if (updatingSDTrajectory) {
      
      //Receive a character from serial if available
      recvData();
      
      //If a full vector has been received, store it on the SD card
      if (newData) {
        newData = false;
        writeVectorToSD();
        returnData();
        Serial.println("ready for vector");
      }

    }
    
    else {
      //starting to run trajectory, need to initialize servos and SD card
      if (trajectoryReset) {
        //Attach servos. Note: Pin 10 was acting strangely, and the servo was being sent false signals. Try to avoid using it in the future.
        R.attach(9);
        L.attach(8);
        U.attach(6);
        D.attach(5);
        //Open file on SD for reading
        sdFile = SD.open("test.txt", O_READ);
        
        //Seek to beginning of SD file
        sdFile.seek(0);
    
        //Reset timestep
        timestep = 0;
    
        //Mark the starting time on the Arduino for the trajectory to keep pace with
        startTime = millis();
    
        trajectoryReset = false;    // next time, no need to perform initialization

        Serial.println("start instr2");
      }
      
      // check if there are vector points available to read
      if (sdFile.available()) {

        //If a full vector has not been read yet, receive more characters from SD card, otherwise parse the data
        if (!newData) {
          receiveChar();
          return;
        } else if (!parsed) {
          //Parse the character data into floats if it hasn't been parsed yet
          timestep   = atof(recvChars[0]);
          yawAngle   = atof(recvChars[1]);
          upAngle    = atof(recvChars[2]);
          downAngle  = atof(recvChars[3]);
          leftAngle  = atof(recvChars[4]);
          rightAngle = atof(recvChars[5]);
  
          //Convert the angle into milliseconds for the servo signal
          //Angle values from MATLAB are set so that zero value corresponds to servo at 90 degrees
          //Slope and initial value of 1500 experimentally determined
          upMS    = (round) (upAngle    * 8.897 + 1500.0);  // servo 1
          downMS  = (round) (downAngle  * 8.818 + 1500.0);  // servo 3
          leftMS  = (round) (leftAngle  * 8.865 + 1500.0);  // servo 2
          rightMS = (round) (rightAngle * 8.818 + 1500.0);  // servo 4

          
          parsed = true;
        }
      }
  
      //If it is the correct time to run the vector, send it to the Arduino
      if (((millis() - startTime) >= timestep * 1000)) {
        Serial.print("up: "); Serial.print(upMS);
        Serial.print("  down: "); Serial.print(downMS);
        Serial.print("  left: "); Serial.print(leftMS);
        Serial.print("  right: "); Serial.print(rightMS);
        Serial.println();
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

        Serial.println("get IMU");  //Now that trajectory point has been performed, MATLAB should get
                                    //current fin orientation via IMU

        //Send data for trajectory point performed and associated times to MATLAB over Serial 
//        Serial.print(millis() - initTime);  Serial.print(" ");
//        Serial.print(millis() - startTime); Serial.print(" ");
//        Serial.print(timestep * 1000, 8);   Serial.print(" ");
//        Serial.print(upAngle);   Serial.print(" ");
//        Serial.print(downAngle); Serial.print(" ");;
//        Serial.print(leftAngle); Serial.print(" ");
//        Serial.print(rightAngle);
//        Serial.println();     //newline character is terminator character in MATLAB
      
        //Mark that the next vector is ready to be received from SD
        newData = false;
        parsed = false;

        if (!sdFile.available()) {
          Serial.println("done instr2");    //tell MATLAB trajectory is done being performed
          waitingForInstruction = true;
          sdFile.close();                   //close SD file now that we are done writing to it
          return;
        }
      }
    } 
  }
}


//Retrieve a character from SD card and put it in recvChars
void receiveChar() {
  received = sdFile.read();
  if (recvInProgress == true) {
    if (received == endMarker) { //End of 3-value pair
      newData = true;
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

    newData = false;
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
      waitingForInstruction = true; //trajectory has been communicated to SD card, now waiting for next instruction
      sdFile.close();               //done writing to SD card
      Serial.println("Done receiving trajectory");
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

void calcEncoderSignal() {
  lastInterruptTime = micros();
  if (digitalRead(PWMPin) == HIGH) {
    timerStart = micros();
  } else if (timerStart != 0) {
    pulseTime = ((volatile int)micros() - timerStart);
    timerStart = 0;
  }
}

float getCentralAngle() {
  return ((float)pulseTime - PWMMin) * (16383.0f / 16384.0f) * 360.0f / (PWMMax - PWMMin);
}

void centralPID() {
  centralAngle = getCentralAngle();
  PIDError = centralAngle - yawAngle;

  currentTime = micros();
  PIDErrorD = (centralAngle - lastCentralAngle) / (currentTime - prevTime);

  centralMS = (round) (1000 + kP*PIDError + kD*PIDErrorD);
  C.writeMicroseconds(centralMS);

  lastCentralAngle = centralAngle;
  prevTime = currentTime;
}