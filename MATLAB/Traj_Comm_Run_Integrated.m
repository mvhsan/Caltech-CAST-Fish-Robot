%The trajectory is broken up into three components: 
%   start-up sequence  - prevents fin from jerking to starting position
%   actual trajectory  - trajectory you actually want to run
%   wind-down sequence - returns fin to original psoition

%% import libraries
dll_location = 'C:\Users\Wind Tunnel\Desktop\fish\james surf\vnproglib\net\bin\win64\VectorNav.dll';

NET.addAssembly(dll_location)

disp("done loading dll")

import VectorNav.Sensor.*
import VectorNav.Protocol.Uart.*
disp("done importing libraries")

%% Prepare trajectory data
%Read timestep and tait-bryan angles from CSV
datas = csvread("trajectorydata/Trajectories/pitch25_roll0.csv");
t = datas(:, 1); %timestep
yaw_datas = datas(:, 2); %yaw angle
pitch_datas = datas(:, 3); %pitch angle
roll_datas = datas(:, 4); %roll angle

pitch_datas = pitch_datas * (25 / 24.6367) * (5/3);
roll_datas  = roll_datas  * (25 / 24.6367) * (5/3);

start_traj_yaw     = yaw_datas(1, 1);
start_traj_pitch   = pitch_datas(1, 1);
start_traj_roll    = roll_datas(1, 1);

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
pitch_roll_values = zeros(num_vectors, 3);

%give IMU time to go to reset point
[UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics(reset_pitch, reset_roll);
current_time = 0;


%linearly interpolate between reset trajectory position and the point the 
%actual trajectory starts at
for pointNumber = 1:NUM_INTERPOLATED_PTS
    interpolated_pitch  =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_pitch) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_pitch;
    interpolated_roll   =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_roll) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_roll;
    interpolated_yaw    =   ((pointNumber - 1) / NUM_INTERPOLATED_PTS * start_traj_yaw) ...
                          + (1 - ((pointNumber - 1) / NUM_INTERPOLATED_PTS)) * reset_yaw ...
                          + 90;
    
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
    pitch_roll_values(pointNumber, 2) = interpolated_pitch;
    pitch_roll_values(pointNumber, 3) = interpolated_roll;
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
        [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics((pitch_datas(i, 1)), (roll_datas(i, 1)));

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
final_traj_pitch   = pitch_datas(end - 1, 1);
final_traj_roll    = roll_datas(end - 1, 1);

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
    pitch_roll_values(vector_pos, 2) = interpolated_pitch;
    pitch_roll_values(vector_pos, 3) = interpolated_roll;

end

disp("done calculating angles");

%% Open Communication With Arduino
delete(instrfindall)            %delete any previous connections
s = serialport('COM4', 115200); %create the serial communication
%set the baudrate (same as the arduino one)

configureCallback(s, "terminator", @RespondToArduino);
configureTerminator(s, "CR/LF");

pause(3);   %allow some buffer time for serial communication setup

global arduinoAcknowledged;     %Arduino has not yet sent message that
arduinoAcknowledged = false;    %MATLAB has acknowledged
global initializedSD;           %SD card has not yet been initialized
initializedSD = false;

%Arduino does not start until MATLAB sends message over Serial
write(s, "MATLAB ready for transmission", "string");

%MATLAB code is trapped until callback function is triggered
while arduinoAcknowledged == false
    pause(.5);
    disp("waiting for SD update");
end

if initializedSD == false   %either SD-fail or SD-success was sent
    disp("SD initialization failed.");
    while true              % need to restart code, SD card failed
    end
end

disp("done opening communication with Arduino");
%% Communicate Trajectory to Arduino
%transform the vector into string with starting ('<'), delimiter (','),
%and ending ('>') characters
vectorStartChar = "<";
vectorDelimiter = ",";
vectorEndChar = ">";
transmissionDoneChar = "!";

vectorSizes = size(vectors);

arduinoAcknowledged = false;    %reset flag
write(s, "instr1", "uint8");    %tell Arduino to switch to mode where trajectory
                                    %is received

%wait for Arduino to acknowledge instruction                        
while arduinoAcknowledged == false
    pause(1);
    disp("waiting to communicate traj");
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
    end

    readyForVector = false;
    write(s, vectorMessage, "uint8");

end

%Send the done character to indicate that transmission of data is done
write(s, transmissionDoneChar, "uint8");

%% Open Communication with IMU

ez = EzAsyncData.Connect('COM3', 115200);
pause(2);
disp("done connecting to IMU")

%% Run Trajectory from Arduino
%While running trajectory from Arduino, get corresponding orientation from
%IMU, store timestamp, pitch and roll in array IMUData
arduinoAcknowledged = false;    %reset flag

global trajectoryStarted;       %set once trajectory starts
trajectoryStarted = false;      %trajectory not yet started
write(s, "instr2", "string");   %tell Arduino to switch to mode where
                                %trajectory stored on SD card is performed

%wait for Arduino to acknowledge instruction
%Arduino will be outputting the servo angles it is writing, and the times
%at which they are doing so
while arduinoAcknowledged == false
    pause(.1)
end

%wait for Arduino to actually start running servo trajectory
while trajectoryStarted == false
    pause(.01)
end

trajectoryStartTime = clock;      %get time where trajectory was started

global getIMUData;      %flag set when IMU data should be sampled
getIMUData = false;

IMUData = zeros(size(vectors, 1), 3);
disp(class(IMUData))
IMUDataRow = 1;

%wait for trajectory to be completed
global trajectoryCompleted;
trajectoryCompleted = false;
while trajectoryCompleted == false
    pause(.001);
    %Arduino has updated trajectory, should get new fin trajectory
    if getIMUData == true   
        getIMUData = false;     %clear flag

        timeElapsed = etime(clock, trajectoryStartTime);  %get current time
        orientation = ez.CurrentData.YawPitchRoll;

        IMUData(IMUDataRow, 1) = timeElapsed;
        disp(timeElapsed)
        IMUData(IMUDataRow, 2) = orientation.Y;     %pitch
        IMUData(IMUDataRow, 3) = orientation.Z;     %roll

        IMUDataRow = IMUDataRow + 1;

    end
    
    IMUData = IMUData(1:IMUDataRow - 1, :); %only keep nonzero IMU values
    
    %we assume that orientation starts with pitch and roll as 0, may not
    %actually occur due to angled surfaces
    
end

IMUData(:, 1) = IMUData(:, 1) - IMUData(1, 1);  %subtract initial pitch
IMUData(:, 2) = IMUData(:, 2) - IMUData(1, 2);  %subtract initial roll
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
plot(IMU_time, (25 / 24.6367) *(5/3) * IMU_pitch);
legend('desired', 'actual');

hold off;

figure();
plot(desired_times, desired_roll);
hold on;
plot(IMU_time, -1 * IMU_roll);
legend('desired', 'actual');

%% disconnect from IMU
ez.Disconnect();
disp("disconnected")

%% Callback Functions

%This callback function is called whenever the Arduino sends MATLAB
%something over serial. It will either print it to the user console, or
%update arduinoAcknowledged so that MATLAB knows the Arduino has sent a 
%specific message.

%must have "serial" and "event" as arguments as part of callback syntax
function RespondToArduino(serial, event)

    %must be global to update variables outside callback function
    global arduinoAcknowledged; %flag for whether Arduino has acknowledged
                                %message via Serial from MATLAB
    global initializedSD;       %flag for whether SD card has been
                                %successfully initialized
    global getIMUData;          %flag for whether orientation data should be
                                %sampled from IMU
    global trajectoryStarted;   %flag for whether Arduino has started
                                %performing trajectory on SD card
    global trajectoryCompleted; %flag for whether trajectory stored on SD
                                %card has been performed by Arduino
    global readyForVector;      %flag for whether Arduino is ready to
                                %receive next trajectory vector from MATLAB
    line = readline(serial);    %get message from Arduino via serial
    if line == "received" || line == "SD-fail"
        arduinoAcknowledged = true;
    elseif line == "SD-success"
        arduinoAcknowledged = true;
        initializedSD = true;
    elseif line == "start instr2"
        trajectoryStarted = true;
    elseif line == "done instr2"
        trajectoryCompleted = true;
        disp("Trajectory has been performed");
    elseif line == "get IMU"
        getIMUData = true;
    elseif line == "ready for vector"
        readyForVector = true;
    else
         disp(line);
    end
end
