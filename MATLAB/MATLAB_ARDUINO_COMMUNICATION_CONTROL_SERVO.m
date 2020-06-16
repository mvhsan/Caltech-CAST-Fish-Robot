%clear all
%close all
%clc

delete(instrfindall)

%------------- Setup Connection with Motor Control Arduino ----------------
serialPort = 'COM3';    % define COM port #
s = serial(serialPort); % create the serial communication
s.Baudrate = 9600; %set the baudrate (same as the arduino one)
fopen(s); %open the communication

pause(3);

vectorSizes = size(vectors);

%fprintf(s, '%s', num2str(vectorSizes(1)));  % send number number of position points
dat = fscanf(s,'%s');

%%
%for loop : sending the datas as a string to arduino 
%position = phi, theta, psi 
%while(1)
    for curVector = 1:1:vectorSizes(1)
        %but because the position loop isn't done for the moment, the first
        %value angles_8(i, 1) must be a pwm between 1000 and 2000 (if you don't 
        %want to go to fast with the continuous rotation servo, choose a value close to 1500
        %close to 1500 (which is the value to stop the servo) for example 1450
        %and 1550.

        %transform the vector into string with starting ('<'), delimiter (','),
        %and ending ('>') characters
        vectorMessage = "<"; % Start vector character

        for curVectorValue = 1:vectorSizes(2)
            vectorMessage = join([vectorMessage num2str(vectors(curVector, curVectorValue), '%.6f') ","], "");
        end
        vectorMessage = strip(vectorMessage,'right', ','); % Remove trailing delimiter
        vectorMessage = join([vectorMessage ">"], ""); % End vector character
        %positionMessage=['<' num2str(time, '%.6f') ',' num2str(position(1), '%.6f') ',' num2str(position(2), '%.6f') ',' num2str(position(3),'%.6f') '>'];
        %send the message to arduino
        fprintf(s, '%s', vectorMessage);
        %read the data sent by the arduino to check if it is correct
        dat = fscanf(s,'%s')

    end
    fprintf(s, '%s', "!");
%end

delete(instrfindall)

