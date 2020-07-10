#include <inttypes.h>
#include <SBGC_Arduino.h>
#include <SBGC.h>

// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

SerialCommand inCmd;
SerialCommand outCmd;

SBGC_cmd_angles_data_t curAngleData;

long startTime;
void setup() {
  Serial1.begin(SERIAL_SPEED);
  Serial.begin(SERIAL_SPEED);
  SBGC_Demo_setup(&Serial1);


  // Take a pause to let gimbal controller to initialize
  delay(3000);

  outCmd.init(SBGC_CMD_GET_ANGLES);
  sbgc_parser.send_cmd(outCmd, 0);
}

void loop() {
    sbgc_parser.send_cmd(outCmd, 0);


  if (sbgc_parser.read_cmd()) {
    //Receive incoming data
    inCmd = sbgc_parser.in_cmd;

    //Unpacks the incoming data into curAngleData
    SBGC_cmd_angles_data_unpack(curAngleData, inCmd);

    //Serial.print("ROLL: ");
    Serial.print(SBGC_ANGLE_TO_DEGREE(curAngleData.sensor_data[0].imu_data));
    Serial.print(",");
    //Serial.print("PITCH: ");
    Serial.print(SBGC_ANGLE_TO_DEGREE(curAngleData.sensor_data[1].imu_data));    Serial.print(",");
    //Serial.print("YAW: ");
    Serial.println(SBGC_ANGLE_TO_DEGREE(curAngleData.sensor_data[2].imu_data));
    // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees

    delay(20);
  }
}
