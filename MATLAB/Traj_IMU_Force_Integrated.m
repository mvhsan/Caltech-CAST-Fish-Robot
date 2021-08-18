%% Description

% This MATLAB script integrates two microcontrollers (Teensy or Arduino)
% a force sensor, and a VectorNav inertial measurement unit (VN-100).
% It is also able to calculate trajectory points and corresponding servo
% angles.

% Sections:
%  Import Libraries: 
%    - libraries for IMU
%  Prepare Trajectory Data:
%    - takes in CSV of desired yaw, pitch and roll values
%    - performs inverse kinematics to get corresponding servo angles for trajectory
%  Open Communication with Servo Teensy
%    - connects to Teensy that controls servos
%    - baud rate 115200
%  Open Communication with Force Sensor Arduino
%    - connects to Arduino that controls power relay for servos and also
%      gets data from force sensor
%    - baud rate 115200
%  Communicate Trajectory to Servo Teensy
%    - sends trajectory vectors to servo Teensy one-by-one, only sends
%      new vector once Teensy is ready
%    - Teensy stores vectors on built-in SD card
%  Open Communication with IMU
%    - connects to VN-100 IMU using VectorNav library
%    - baud rate 115200
%  Run Trajectory from Servo Teensy
%    - instructs Teensy to run trajectory saved on built-in SD card
%    - Teensy instructs MATLAB to get IMU data after each trajectory
%      vector, IMU data collection synchronized with force sensor data
%      collection

%% Prepare trajectory data
% The trajectory is broken up into three components: 
%   start-up sequence  - prevents fin from jerking to starting position
%   actual trajectory  - trajectory you actually want to run
%   wind-down sequence - returns fin to original position

%Read timestep and tait-bryan angles from CSV
datas = csvread("C:\Users\Wind Tunnel\Desktop\fish\james surf\trajectorydata\Trajectories\Gen_0_C_1_MaxAng_15_ThkAng_15_RotAng_15_RotInt_15_SpdCde_0_Spdupv_1.3_Kv_0_hdev_1_freq_0.4_TB.csv");
t = datas(:, 1); %timestep
yaw_datas = datas(:, 2); %yaw angle
pitch_datas = datas(:, 3); %pitch angle
roll_datas = datas(:, 4); %roll angle

angle_multiplier = 15 / 14.167; %experimentally determined multiplier
                                %difference between actual and desired
                                %trajectory

adjusted_pitch_datas = pitch_datas * angle_multiplier;
adjusted_roll_datas  = roll_datas  * angle_multiplier;

start_traj_yaw     = yaw_datas(1, 1);
start_traj_pitch   = adjusted_pitch_datas(1, 1);
start_traj_roll    = adjusted_roll_datas(1, 1);
actual_start_traj_pitch  = pitch_datas(1, 1);
actual_start_traj_roll   = roll_datas(1, 1);

reset_yaw   = 90;
reset_pitch = 0;
reset_roll  = 0;

%this is the "start-up" sequence, the fin starts pointing straight "up,"
%and gradually transitions to the first actual trajectory point
NUM_INTERPOLATED_PTS    = 50;   %number of discrete points for start-up
START_TIME              = 2;    %time allotted for start-up sequence in seconds

NUM_CYCLES              = 3;    %number of times "actual" trajectory is
                                %performed
%Initialize output vectors
num_vectors       = NUM_CYCLES * (size(datas, 1) - 1) + 2 * NUM_INTERPOLATED_PTS;
vectors           = zeros(num_vectors, 6);
pitch_roll_values = zeros(num_vectors, 3);  %table containing pitch and roll
                                            %values performed by fin

%give IMU time to go to reset point
[UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics(reset_pitch, reset_roll);
current_time = 0;


%linearly interpolate between reset trajectory position and the point the 
%actual trajectory starts at
for pointNumber = 1:NUM_INTERPOLATED_PTS
    %use (pointNumber - 1) so that pointNumber #1 corresponds to reset
    interpolated_pitch  =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_pitch) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_pitch;
    interpolated_roll   =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_roll) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_roll;
    interpolated_yaw    =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_yaw) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_yaw ...
                          + 90;     %yaw is given from -90 to 90 but we
                                    %want it from 0 to 180

    actual_pitch  =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * actual_start_traj_pitch) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_pitch;
    actual_roll   =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * actual_start_traj_roll) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_roll;

    %solve inverse kinematics for interpolated angles
    %yaw angle is directly performed using separate motor, not parallel servos
    %for fin, so inverse kinematics not needed
    [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics(interpolated_pitch, interpolated_roll);
    
    %first column of vectors is desired time, pointNumber starts at 1 but
    %time should start at 0, so (pointNumber - 1) is used
    
    vector_pos              = pointNumber;  %account for reset point
    current_time            = (pointNumber - 1) / NUM_INTERPOLATED_PTS * START_TIME;
    vectors(pointNumber, 1) = current_time;
    vectors(pointNumber, 2) = interpolated_yaw;
    vectors(pointNumber, 3) = UAngle;
    vectors(pointNumber, 4) = DAngle;
    vectors(pointNumber, 5) = LAngle;
    vectors(pointNumber, 6) = RAngle;
    
    pitch_roll_values(pointNumber, 1) = current_time;
    pitch_roll_values(pointNumber, 2) = actual_pitch;
    pitch_roll_values(pointNumber, 3) = actual_roll;
end

%get number of points in actual trajectory
[num_points, ~] = size(datas);

%Process and add data to output vectors
for cycleNumber = 1:1:NUM_CYCLES
    
    %We are opting to use extra memory on the SD card, rather than spend
    %extra time processing on the Arduino. Otherwise, the Arduino would
    %have to determine where in the SD card to look within each cycle, so
    %that it executes either the "start path," "end path," or trajectory
    %data.
    for i = 1:1:(num_points - 1)    %Last point same as first point
        %position in vectors to write to depends on size of "start" code,
        %number of cycles that have been written already, and position in
        %current cycle
        vector_pos = (cycleNumber - 1) * (num_points - 1) + NUM_INTERPOLATED_PTS + i;
        
        %time elapsed is sum of duration of start-up sequence, time for
        %previously completed cycles of actual trajectory, and time in
        %current cycle of actual trajectory
        current_time = (cycleNumber - 1) * t(end, 1) + START_TIME + t(i, 1);
        
        vectors(vector_pos, 1) = current_time;
        vectors(vector_pos, 2) = yaw_datas(i, 1) + 90;

        %Solve inverse kinematics for given desired angles
        [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics((adjusted_pitch_datas(i, 1)), (adjusted_roll_datas(i, 1)));

        vectors(vector_pos, 3) = UAngle;
        vectors(vector_pos, 4) = DAngle;
        vectors(vector_pos, 5) = LAngle;
        vectors(vector_pos, 6) = RAngle;
        
        pitch_roll_values(vector_pos, 1) = current_time;
        pitch_roll_values(vector_pos, 2) = pitch_datas(i, 1);
        pitch_roll_values(vector_pos, 3) = roll_datas(i, 1);
    end
end

%get yaw, pitch and roll for the final point performed in the "actual"
%trajectory
final_traj_yaw     = yaw_datas(end - 1, 1);
final_traj_pitch   = adjusted_pitch_datas(end - 1, 1);
final_traj_roll    = adjusted_roll_datas(end - 1, 1);

actual_final_traj_pitch = pitch_datas(end - 1, 1);
actual_final_traj_roll  = roll_datas(end - 1, 1);

%linearly interpolate between reset trajectory position and the point the 
%actual trajectory ends at
for pointNumber = 1:NUM_INTERPOLATED_PTS
    interpolated_pitch  = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_pitch ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_pitch;
    interpolated_roll   = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_roll ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_roll;
    interpolated_yaw    = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_yaw ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_yaw ...
                          + 90;     %add 90 since "zero line" is actually 90
                      
    actual_pitch  = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * actual_final_traj_pitch ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_pitch;
    actual_roll   = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * actual_final_traj_roll ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_roll;
  
    
    [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics(interpolated_pitch, interpolated_roll);
    
    current_time =   START_TIME + cycleNumber * t(end, 1) ...
                   + (pointNumber - 1) / NUM_INTERPOLATED_PTS * START_TIME;

    vector_pos = NUM_INTERPOLATED_PTS + (num_points - 1) * NUM_CYCLES + pointNumber;
    vectors(vector_pos, 1)  = current_time;
    vectors(vector_pos, 2)  = interpolated_yaw;
    vectors(vector_pos, 3)  = UAngle;
    vectors(vector_pos, 4)  = DAngle;
    vectors(vector_pos, 5)  = LAngle;
    vectors(vector_pos, 6)  = RAngle;
    
    pitch_roll_values(vector_pos, 1) = current_time;
    pitch_roll_values(vector_pos, 2) = actual_pitch;
    pitch_roll_values(vector_pos, 3) = actual_roll;

end

disp("done calculating angles");

%% Open Communication With Servo Teensy
delete(instrfindall)            %delete any previous connections
servo_port = serialport('COM6', 115200); %create the serial communication
%set the baudrate (same rate as on Teensy)

%configureCallback(servo_port, "terminator", @RespondToServoArduino);
configureTerminator(servo_port, "CR/LF");

pause(3);   %allow some buffer time for serial communication setup

%Teensy has not yet sent message that MATLAB has acknowledged
teensyAcknowledged = false;            

%Teensy does not start until MATLAB sends message over Serial
write(servo_port, "MATLAB ready for transmission", "string");

while teensyAcknowledged == false
    pause(.01);
    if servo_port.NumBytesAvailable() > 0
        teensyMsg = readline(servo_port);
        disp(teensyMsg);
        if contains(teensyMsg, "SD-success")
            teensyAcknowledged = true;
        elseif contains(teensyMsg, "SD-fail")
            disp("SD initialization failed.");
            while true      %forces user to restart code after SD
            end             %initialization failed
        end
    end    
end

disp("done opening communication with servo Teensy");

%% Open Communication With Force Sensor Arduino
force_port = serialport('COM5', 115200);    %create serial communication
configureTerminator(force_port, "CR/LF");

pause(3);       %allow serial connection to set up

%Arduino does not start until MATLAB sends message over Serial
write(force_port, "MATLAB ready for transmission", "string");
arduinoAcknowledged = false;                %reset flag
while arduinoAcknowledged == false
    pause(.01);
    if force_port.NumBytesAvailable() > 0
        arduinoMsg = readline(force_port);
        disp(arduinoMsg);
        if arduinoMsg == "serial start"
            disp("connection formed with force sensor Arduino");
            arduinoAcknowledged = true;     %set flag
        end
    end    
end

%% Communicate Trajectory to Servo Teensy
%transform the vector into string with starting ('<'), delimiter (','),
%and ending ('>') characters
vectorStartChar = "<";
vectorDelimiter = ",";
vectorEndChar = ">";
transmissionDoneChar = "!";

vectorSizes = size(vectors);

arduinoAcknowledged = false;    %reset flag
write(servo_port, "instr1", "uint8");    %tell Arduino to switch to mode where trajectory
                                    %is received

%wait for Arduino to acknowledge instruction                        
while arduinoAcknowledged == false
    pause(1);
    disp("waiting to communicate traj");
    if servo_port.NumBytesAvailable() > 0
        arduinoMsg = readline(servo_port);
        disp(arduinoMsg);
        if arduinoMsg == "received"
            arduinoAcknowledged = true;
        end
    end    
end

global readyForVector;      %flag for whether Arduino is ready for
readyForVector = true;     %next vector

%for loop : sending the datas as a string("<yaw, pitch, roll>") to Arduino
%Arduino will be sending back vectors received, which will be displayed
%in user console for troubleshooting purposes.
for curVector = 1:1:vectorSizes(1)
    vectorMessage = vectorStartChar; % Running vector message to hold a single vector

    for curVectorValue = 1:1:vectorSizes(2)
        vectorMessage = join([vectorMessage num2str(vectors(curVector, curVectorValue), '%.6f') vectorDelimiter], "");
    end
    vectorMessage = strip(vectorMessage,'right', vectorDelimiter); % Remove trailing delimiter
    vectorMessage = join([vectorMessage vectorEndChar], ""); % End vector character
    %disp(vectorMessage)
    %send the message to arduino
    
    while readyForVector == false
        pause(.01);
        if servo_port.NumBytesAvailable() > 0
            arduinoMsg = readline(servo_port);
            if arduinoMsg == "ready for vector"
                readyForVector = true;
            else
                disp(arduinoMsg);
            end
            
        end   
    end

    readyForVector = false;
    write(servo_port, vectorMessage, "uint8");

end

%Send the done character to indicate that transmission of data is done
write(servo_port, transmissionDoneChar, "uint8");

disp("done communicating trajectory");

%% Open Communication with IMU

dll_location = 'C:\Users\Wind Tunnel\Desktop\fish\james surf\vnproglib\net\bin\win64\VectorNav.dll';

NET.addAssembly(dll_location)

disp("done loading dll")

import VectorNav.Sensor.*
import VectorNav.Protocol.Uart.*
disp("done importing libraries")

ez = EzAsyncData.Connect('COM9', 115200);
pause(2);
disp("done connecting to IMU")

%% Run Trajectory from Servo Teensy
%While running trajectory from Teensy, get corresponding orientation from
%IMU, store timestamp, pitch and roll in array IMUData
teensyAcknowledged = false;    %reset flag

trajectoryStarted = false;      %set once trajectory starts
write(servo_port, "instr2", "string");   %tell Teensy to switch to mode where
                                %trajectory stored on SD card is performed

%wait for Teensy to acknowledge instruction
%Teensy will be outputting the servo angles it is writing, and the times
%at which they are doing so
while teensyAcknowledged == false
    if servo_port.NumBytesAvailable() > 0
        teensyMsg = readline(servo_port);
        if teensyMsg == "received"
            teensyAcknowledged = true;
        end
    end
end

%wait for Teensy to actually start running servo trajectory
while trajectoryStarted == false
    if servo_port.NumBytesAvailable() > 0
        teensyMsg = readline(servo_port);
        if teensyMsg == "start instr2"
            trajectoryStarted = true;
        end
    end
end
disp("start trajectory");
trajectoryStartTime = clock;      %get time where trajectory was started

IMUData      = zeros(size(vectors, 1), 3);  %holds time, pitch, roll
IMUDataRow   = 1;
ForceData    = zeros(size(vectors, 1), 2);  %holds time and force
ForceDataRow = 1;

%keep running until trajectory is completed
trajectoryCompleted = false;            %flag set when trajectory completed
while trajectoryCompleted == false
    pause(.001);
    
    %check for Serial data from servo Teensy
    if servo_port.NumBytesAvailable() > 0
        teensyMsg = readline(servo_port);
        if teensyMsg == "done instr2"
            trajectoryCompleted = true;
        elseif teensyMsg == "get IMU"
            timeElapsed = etime(clock, trajectoryStartTime);  %get current time
            orientation = ez.CurrentData.YawPitchRoll;

            IMUData(IMUDataRow, 1) = timeElapsed;
            IMUData(IMUDataRow, 2) = orientation.Y;     %pitch
            IMUData(IMUDataRow, 3) = orientation.Z;     %roll

            IMUDataRow = IMUDataRow + 1;    %increment row
            
            %get force sensor data
            write(force_port, "get force", "string");
        end
    end
    
    %check for Serial data from force sensor Arduino
    if force_port.NumBytesAvailable() > 0
        arduinoMsg = readline(force_port);
        if arduinoMsg == "FORCE ERROR"
            disp("force has exceeded threshold amount");
            trajectoryCompleted = true;
        else
            force_reading = str2double(arduinoMsg);
            timeElapsed = etime(clock, trajectoryStartTime);  %get current time
            
            ForceData(ForceDataRow, 1) = timeElapsed;
            ForceData(ForceDataRow, 2) = force_reading;
            
            ForceDataRow = ForceDataRow + 1;    %increment row
        end
    end
    
    
end

ForceData = ForceData(1:ForceDataRow - 1, :); %trim off excess rows

IMUData = IMUData(1:IMUDataRow - 1, :); %only keep nonzero IMU values

%we assume that orientation starts with pitch and roll as 0, may not
%actually occur due to angled surfaces

IMUData(:, 2) = IMUData(:, 2) - IMUData(1, 2);  %subtract initial pitch
IMUData(:, 3) = IMUData(:, 3) - IMUData(1, 3);  %subtract initial roll

disp("done completing trajectory");
%% Generating plots

desired_times = pitch_roll_values(:, 1);
desired_pitch = pitch_roll_values(:, 2);
desired_roll  = pitch_roll_values(:, 3);

IMU_time      = IMUData(:, 1);
IMU_pitch     = IMUData(:, 2);
IMU_roll      = IMUData(:, 3);

figure();
plot(desired_times, desired_pitch);
hold on;
plot(IMU_time, -1 * IMU_pitch);
legend('expected', 'actual');
title("Actual versus expected pitch values for selected trajectory")
xlabel("Time Elapsed (seconds)");
ylabel("Angle (degrees)");
ylim([-20 20])

hold off;

figure();
plot(desired_times, desired_roll);
hold on;
plot(IMU_time, IMU_roll);
legend('expected', 'actual');
title("Actual versus expected roll values for selected trajectory")
xlabel("Time Elapsed (seconds)");
ylabel("Angle (degrees)");
ylim([-20 20])


%% Other plots

figure();
plot(desired_times, desired_pitch);
hold on;
plot(IMU_time, -1 * IMU_roll);

plot(desired_times, desired_roll);
plot(IMU_time, -IMU_pitch);
legend('desired pitch', 'actual pitch', 'desired roll', 'actual roll');
hold off;

%% disconnect from IMU
ez.Disconnect();
disp("disconnected")

