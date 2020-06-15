saveFolder='C:\Postdoc\Fish robot\Continuous rotation\FlexibleForked\Drag\40Hz';
filename='1300';
comPortForce = 'COM6';
comPortPos = 'COM8';
TimeStep = 100;
nPoints = 200; 

data=0;
sForce = serial(comPortForce); 
sPos = serial(comPortPos);

sForce.Baudrate = 9600; 
sForce.Timeout = 5*TimeStep/1000;
fopen(sForce);

sPos.Baudrate = 9600; 
sPos.Timeout = 5*TimeStep/1000;
fopen(sPos);

pause(5)

fprintf(sForce, '%d\2', [nPoints TimeStep]);
fprintf(sPos, '%d\2', [nPoints TimeStep]);

tForce=zeros(1,nPoints);
force=zeros(1,nPoints);
force=single(force);
tPos=zeros(1,nPoints);
position=zeros(1,nPoints);
position=single(force);
count=1;

while count < nPoints+1
    
    tForce(count) = fscanf(sForce,'%i');
    force(count) = fscanf(sForce,'%f');
    tPos(count) = fscanf(sPos,'%i');
    position(count) = fscanf(sPos,'%f');
    count=count+1;
    
end

% forceN=(2.2375*force-127.1903)*9.81/1000; %From fit to calibration

save(strcat(saveFolder,'\',filename,'_force.mat'),'tForce','force');
save(strcat(saveFolder,'\',filename,'_pos.mat'),'tPos','position');
fclose(sForce);
fclose(sPos);
h=msgbox('Finished');