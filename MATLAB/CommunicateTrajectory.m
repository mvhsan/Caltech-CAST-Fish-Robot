%% Prepare trajectory data
%Read timestep and tait-bryan angles from CSV
datas = csvread("trajectorydata/Trajectories/Gen_0_C_1_MaxAng_18.6_ThkAng_13.5_RotAng_36_RotInt_4.6_SpdCde_1_Spdupv_0.8_Kv_0.2_hdev_0.9_freq_0.1_TB.csv");

t = datas(:, 1); %timestep
yaw_datas = datas(:, 2); %yaw angle
pitch_datas = datas(:, 3); %pitch angle
roll_datas = datas(:, 4); %roll angle

%Initialize output vectors
vectors = zeros(size(datas, 1), 6);

%Process and add data to output vectors
for i = 1:1:size(datas)
    vectors(i, 1) = t(i, 1);
    vectors(i, 2) = yaw_datas(i, 1) + 90;
    
    %Solve inverse kinematics for given desired angles
    [UAngle, DAngle, LAngle, RAngle] = solveInverseKinematics((pitch_datas(i, 1)), (roll_datas(i, 1)));
    
    vectors(i, 3) = UAngle;
    vectors(i, 4) = DAngle;
    vectors(i, 5) = LAngle;
    vectors(i, 6) = RAngle;
end
%% Setup Connection with  Arduino
delete(instrfindall) %Delete any previous connections

serialPort = 'COM3';    % define COM port #
s = serial(serialPort); % create the serial communication
s.Baudrate = 250000; %set the baudrate (same as the arduino one)
fopen(s); %open the communication

pause(3);

%Check if overwriting is necessary
serialdata = fscanf(s, '%s');
while (true)
    if (serialdata == ">>>") %Arduino requires input
        userinput = input('Overwrite? (Y/N): ', 's');
        fprintf(s, '%s', userinput);
    end
    if (serialdata == "<<<") %Arduino is ready for data transmission
        break;
    end
    serialdata = fscanf(s, '%s')
end
%% Communicate data to Arduino
%transform the vector into string with starting ('<'), delimiter (','), and ending ('>') characters
vectorStartChar = '<';
vectorDelimiter = ',';
vectorEndChar = '>';
transmissionDoneChar = '!';

%for loop : sending the datas as a string("<yaw, pitch, roll>") to arduino 
for curVector = 1:1:size(vectors)
    vectorMessage = '' + vectorStartChar; % Running vector message to hold a single vector

    for curVectorValue = 1:vectorSizes(2)
        vectorMessage = join([vectorMessage num2str(vectors(curVector, curVectorValue), '%.6f') vectorDelimiter], "");
    end
    vectorMessage = strip(vectorMessage,'right', vectorDelimiter); % Remove trailing delimiter
    vectorMessage = join([vectorMessage vectorEndChar], ""); % End vector character
    %positionMessage=['<' num2str(time, '%.6f') ',' num2str(position(1), '%.6f') ',' num2str(position(2), '%.6f') ',' num2str(position(3),'%.6f') '>'];
    %send the message to arduino
    fprintf(s, '%s', vectorMessage);
    %read the data sent by the arduino to check if it is correct
    dat = fscanf(s,'%s')

end
fprintf(s, '%s', transmissionDoneChar); %Done character

delete(instrfindall)