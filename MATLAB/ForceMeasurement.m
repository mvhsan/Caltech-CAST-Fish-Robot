saveFolder='forcedata';
weight=0;
comPort = 'COM8';
TimeStep = 200;
nPoints = 50; 

data=0;
s = serial(comPort); 
s.Baudrate = 9600; 
s.Timeout = 5*TimeStep/1000;
fopen(s);

pause(5)

disp("done setting up communication");

fprintf(s, '%d\2', [nPoints TimeStep]);
t=zeros(1,nPoints);
force=zeros(1,nPoints);
force=single(force);
count=1;

while count < nPoints+1
    
    t(count) = fscanf(s,'%i');
    force(count) = fscanf(s,'%f');
    count=count+1;
    
end

forceN=(2.2375*force-127.1903)*9.81/1000; %From fit to calibration

save(strcat(saveFolder,'\real',num2str(weight),'g.mat'),'t','force');
fclose(s);
h=msgbox('Finished');