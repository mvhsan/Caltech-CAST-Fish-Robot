#include <inttypes.h>
#include <SBGC_Arduino.h>
#include <SBGC.h>

// Serial baud rate should match with the rate, configured for the SimpleBGC controller
#define SERIAL_SPEED 115200

// delay between commands, ms
#define SBGC_CMD_DELAY 20

//SBGC_Parser sbgc_parser;  // SBGC command parser. Define one for each port.
//ArduinoComObj com_obj; // COM-port wrapper required for parser

SerialCommand inCmd;
SerialCommand outCmd;

SBGC_cmd_realtime_data_t curRTData;

long startTime;
void setup() {
  Serial1.begin(SERIAL_SPEED);
  Serial.begin(SERIAL_SPEED);
  SBGC_Demo_setup(&Serial1);


  // Take a pause to let gimbal controller to initialize
  delay(3000);

  outCmd.init(SBGC_CMD_REALTIME_DATA_3);
  sbgc_parser.send_cmd(outCmd, 0);


  startTime = millis();
}

void loop() {
  //if (millis() - startTime > 100) {
    sbgc_parser.send_cmd(outCmd, 0);
    //startTime = millis();
  //}

  if (sbgc_parser.read_cmd()) {
    //Receive incoming data
    inCmd = sbgc_parser.in_cmd;

    //Unpacks the incoming data into curRTData
    SBGC_cmd_realtime_data_unpack(curRTData, inCmd);

    /*
      Data stored in curRTData:
      sensor_data[3];  // ACC and Gyro sensor data (with calibration) for current IMU (see cur_imu field)
      int16_t serial_error_cnt; // counter for communication errors
      int16_t system_error; // system error flags, defined in SBGC_SYS_ERR_XX
      uint8_t reserved1[4];
      int16_t rc_raw_data[SBGC_RC_NUM_CHANNELS]; // RC signal in 1000..2000 range for ROLL, PITCH, YAW, CMD, EXT_ROLL, EXT_PITCH channels
      int16_t imu_angle[3]; // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
      int16_t frame_imu_angle[3]; // ROLL, PITCH, YAW Euler angles of a frame, if known
      int16_t target_angle[3]; // ROLL, PITCH, YAW target angle
      uint16_t cycle_time_us; // cycle time in us. Normally should be 800us
      uint16_t i2c_error_count; // I2C errors counter
      uint8_t reserved2;
      uint16_t battery_voltage; // units 0.01 V
      uint8_t state_flags1; // bit0: motor ON/OFF state;  bits1..7: reserved
      uint8_t cur_imu; // actually selecteted IMU for monitoring. 1: main IMU, 2: frame IMU
      uint8_t cur_profile; // active profile number starting from 0
      uint8_t motor_power[3]; // actual motor power for ROLL, PITCH, YAW axis, 0..255

      // Fields below are filled only for CMD_REALTIME_DATA_4 command
      int16_t rotor_angle[3]; // relative angle of each motor, 16384/360 degrees
      uint8_t reserved3;
      int16_t balance_error[3]; // error in balance. Ranges from -512 to 512,  0 means perfect balance.
      uint16_t current; // Current that gimbal takes, in mA.
      int16_t magnetometer_data[3]; // magnetometer sensor data (with calibration)
      int8_t  imu_temp_celcius;  // temperature measured by the main IMU sensor, in Celsius
      int8_t  frame_imu_temp_celcius;  // temperature measured by the frame IMU sensor, in Celsius
      uint8_t reserved4[38];
    */

    //Serial.print(millis()-startTime);
    //Serial.print(",");

    //Serial.print("ROLL: ");
    Serial.print(SBGC_ANGLE_TO_DEGREE(curRTData.imu_angle[0]));
    Serial.print(",");
    //Serial.print("PITCH: ");
    Serial.print(SBGC_ANGLE_TO_DEGREE(curRTData.imu_angle[1]));
    Serial.print(",");
    //Serial.print("YAW: ");
    Serial.println(SBGC_ANGLE_TO_DEGREE(curRTData.imu_angle[2]));

    // ROLL, PITCH, YAW Euler angles of a camera, 16384/360 degrees
  }
}
