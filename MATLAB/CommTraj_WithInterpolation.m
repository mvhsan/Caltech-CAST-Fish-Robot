%% Prepare trajectory data
%Read timestep and tait-bryan angles from CSV
datas = csvread("trajectorydata/Trajectories/Gen_0_C_1_MaxAng_25_ThkAng_25_RotAng_25_RotInt_25_SpdCde_0_Spdupv_0_Kv_0_hdev_0_freq_0.4_TB.csv");
t = datas(:, 1); %timestep
yaw_datas = datas(:, 2); %yaw angle
pitch_datas = datas(:, 3); %pitch angle
roll_datas = datas(:, 4); %roll angle

%Initialize output vectors
vectors = zeros(size(datas, 1), 6);

start_traj_yaw     = yaw_datas(1, 1);
start_traj_pitch   = pitch_datas(1, 1);
start_traj_roll    = roll_datas(1, 1);

reset_yaw   = 90;
reset_pitch = 0;
reset_roll  = 0;

NUM_INTERPOLATED_PTS    = 50;
START_TIME              = 2;            %measured in seconds
NUM_CYCLES              = 3;

for pointNumber = 1:NUM_INTERPOLATED_PTS
    interpolated_pitch  =   (pointNumber / NUM_INTERPOLATED_PTS * start_traj_pitch) ...
                          + (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * reset_pitch;
    interpolated_roll   =   (pointNumber / NUM_INTERPOLATED_PTS * start_traj_roll) ...
                          + (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * reset_roll;
    interpolated_yaw    =   (pointNumber / NUM_INTERPOLATED_PTS * start_traj_yaw) ...
                          + (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * reset_yaw ...
                          + 90;
                      
    [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics(interpolated_pitch, interpolated_roll);
    
    % time starts at zero, which is why we consider (pointNumber - 1)
    vectors(pointNumber, 1) = (pointNumber - 1) / NUM_INTERPOLATED_PTS * START_TIME;
    vectors(pointNumber, 2) = interpolated_yaw;
    vectors(pointNumber, 3) = UAngle;
    vectors(pointNumber, 4) = DAngle;
    vectors(pointNumber, 5) = LAngle;
    vectors(pointNumber, 6) = RAngle;
    
end

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
        
        current_time = (cycleNumber - 1) * t(end, 1) + START_TIME + t(i, 1);
        
        
        vectors(vector_pos, 1) = current_time;
        vectors(vector_pos, 2) = yaw_datas(i, 1) + 90;

        %Solve inverse kinematics for given desired angles
        [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics((pitch_datas(i, 1)), (roll_datas(i, 1)));

        vectors(vector_pos, 3) = UAngle;
        vectors(vector_pos, 4) = DAngle;
        vectors(vector_pos, 5) = LAngle;
        vectors(vector_pos, 6) = RAngle;
    end
end

final_traj_yaw     = yaw_datas(end - 1, 1);
final_traj_pitch   = pitch_datas(end - 1, 1);
final_traj_roll    = roll_datas(end - 1, 1);

for pointNumber = 1:NUM_INTERPOLATED_PTS
    interpolated_pitch  =   (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_pitch ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_pitch;
    interpolated_roll   = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_roll ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_roll;
    interpolated_yaw    = (1 - (pointNumber / NUM_INTERPOLATED_PTS)) * final_traj_yaw ...
                          + (pointNumber / NUM_INTERPOLATED_PTS) * reset_yaw ...
                          + 90;
    
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

end

disp("done calculating angles");
%% Setup Connection with  Arduino
disp("setting up connection");
delete(instrfindall) %Delete any previous connections

serialPort = 'COM7';    % define COM port
s = serial(serialPort); % create the serial communication
s.Baudrate = 115200; %set the baudrate (same as the arduino one)
fopen(s); %open the communication

pause(3);

fprintf(s, '%s', "MATLAB ready for transmission");
    dat = fscanf(s,'%s')
    dat = fscanf(s,'%s')


% %Check if overwriting is necessary
% serialdata = fscanf(s, '%s');
% while (true)
%     if (serialdata == ">>>") %Arduino requires input
%         userinput = input('Overwrite? (Y/N): ', 's');
%         fprintf(s, '%s', userinput);
%     end
%     if (serialdata == "<<<") %Arduino is ready for data transmission
%         break;
%     end
%     serialdata = fscanf(s, '%s')
% end

%% Communicate data to Arduino
%transform the vector into string with starting ('<'), delimiter (','), and ending ('>') characters
vectorStartChar = "<";
vectorDelimiter = ",";
vectorEndChar = ">";
transmissionDoneChar = "!";

vectorSizes = size(vectors);

%for loop : sending the datas as a string("<yaw, pitch, roll>") to arduino 
for curVector = 1:1:vectorSizes(1)
    vectorMessage = vectorStartChar; % Running vector message to hold a single vector

    for curVectorValue = 1:1:vectorSizes(2)
        vectorMessage = join([vectorMessage num2str(vectors(curVector, curVectorValue), '%.6f') vectorDelimiter], "");
    end
    vectorMessage = strip(vectorMessage,'right', vectorDelimiter); % Remove trailing delimiter
    vectorMessage = join([vectorMessage vectorEndChar], ""); % End vector character
    
    %send the message to arduino
    fprintf(s, '%s', vectorMessage);
    
    %read the data sent by the arduino to check if it is correct
    dat = fscanf(s,'%s')
end

%Send the done character to indicate that transmission of data is done
fprintf(s, '%s', transmissionDoneChar); 

delete(instrfindall)