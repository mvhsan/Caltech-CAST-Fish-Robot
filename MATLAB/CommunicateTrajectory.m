%% Prepare trajectory data
%Read timestep and tait-bryan angles from CSV
datas = csvread("trajectorydata/Trajectories/all_zeroes.csv");

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
disp("done calculating angles");
%% Setup Connection with  Arduino
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