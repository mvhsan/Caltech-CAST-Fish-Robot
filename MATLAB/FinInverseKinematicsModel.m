%% 
close all
%% 
%Lengths and variables in inches
baseLength = 1.64;
servoArmLength = .75;
linkageLength = 1.85;
swashPlateLength = 2.43;
middleHeight = 1.8;
%% 
% Start with a blank rigid body tree model.
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);
%% 
%Initialize Base and revolute joints
%Center
middle = robotics.RigidBody('middle');
middleJoint = robotics.Joint('middle', 'fixed');
setFixedTransform(middleJoint,trvec2tform([0 0 middleHeight]));
middle.Joint = middleJoint;

%Left Servo Offset
LServoOffset = robotics.RigidBody('LServoOffset');
LServoOffsetJoint = robotics.Joint('LServoOffsetJoint', 'revolute');
LServoOffsetJoint.HomePosition = pi/6;
LServoOffsetJoint.PositionLimits = [-pi/2 pi/2];
setFixedTransform(LServoOffsetJoint,trvec2tform([-baseLength/2 0 0]));
LServoOffsetJoint.JointAxis = [0 1 0];
LServoOffset.Joint = LServoOffsetJoint;

%Right Servo Offset
RServoOffset = robotics.RigidBody('RServoOffset');
RServoOffsetJoint = robotics.Joint('RServoOffsetJoint', 'revolute');
RServoOffsetJoint.HomePosition = pi/6;
RServoOffsetJoint.PositionLimits = [-pi/2 pi/2];
setFixedTransform(RServoOffsetJoint,trvec2tform([baseLength/2 0 0]));
RServoOffsetJoint.JointAxis = [0 -1 0];
RServoOffset.Joint = RServoOffsetJoint;

%Up Servo Offset
UServoOffset = robotics.RigidBody('UServoOffset');
UServoOffsetJoint = robotics.Joint('UServoOffsetJoint', 'revolute');
UServoOffsetJoint.HomePosition = pi/6;
UServoOffsetJoint.PositionLimits = [-pi/2 pi/2];
setFixedTransform(UServoOffsetJoint,trvec2tform([0 baseLength/2 0]));
UServoOffsetJoint.JointAxis = [1 0 0];
UServoOffset.Joint = UServoOffsetJoint;

%Down Servo Offset
DServoOffset = robotics.RigidBody('DServoOffset');
DServoOffsetJoint = robotics.Joint('DServoOffsetJoint', 'revolute');
DServoOffsetJoint.HomePosition = pi/6;
DServoOffsetJoint.PositionLimits = [-pi/2 pi/2];
setFixedTransform(DServoOffsetJoint,trvec2tform([0 -baseLength/2 0]));
DServoOffsetJoint.JointAxis = [-1 0 0];
DServoOffset.Joint = DServoOffsetJoint;

addBody(robot,middle, 'base');
addBody(robot,LServoOffset, 'base');
addBody(robot,RServoOffset, 'base');
addBody(robot,UServoOffset, 'base');
addBody(robot,DServoOffset, 'base');
%% 
%Initialize servo lever arms

%Left Servo Arm
LServoArm = robotics.RigidBody('LServoArm');
LServoJoint = robotics.Joint('LServoJoint', 'revolute');
LServoJoint.HomePosition = pi/2;
setFixedTransform(LServoJoint,trvec2tform([-servoArmLength 0 0]));
LServoJoint.JointAxis = [0 -1 0];
LServoArm.Joint = LServoJoint;

%Right Servo Arm
RServoArm = robotics.RigidBody('RServoArm');
RServoJoint = robotics.Joint('RServoJoint', 'revolute');
RServoJoint.HomePosition = pi/2;
setFixedTransform(RServoJoint,trvec2tform([servoArmLength 0 0]));
RServoJoint.JointAxis = [0 -1 0];
RServoArm.Joint = RServoJoint;

%Up Servo Arm
UServoArm = robotics.RigidBody('UServoArm');
UServoJoint = robotics.Joint('UServoJoint', 'revolute');
UServoJoint.HomePosition = pi/2;
setFixedTransform(UServoJoint,trvec2tform([0 servoArmLength 0]));
UServoJoint.JointAxis = [1 0 0];
UServoArm.Joint = UServoJoint;

%Down Servo Arm
DServoArm = robotics.RigidBody('DServoArm');
DServoJoint = robotics.Joint('DServoJoint', 'revolute');
DServoJoint.HomePosition = pi/2;
setFixedTransform(DServoJoint,trvec2tform([0 -servoArmLength 0]));
DServoJoint.JointAxis = [1 0 0];
DServoArm.Joint = DServoJoint;

addBody(robot,LServoArm, 'LServoOffset');
addBody(robot,RServoArm, 'RServoOffset');
addBody(robot,UServoArm, 'UServoOffset');
addBody(robot,DServoArm, 'DServoOffset');

%% 
%Initialize Linkages

% Left linkage
LLinkage = robotics.RigidBody('LLinkage');
LLinkageJoint = robotics.Joint('LLinkageJoint', 'revolute');
LLinkageJoint.HomePosition = pi/4;
setFixedTransform(LLinkageJoint,trvec2tform([linkageLength 0 0]));
LLinkageJoint.JointAxis = [0 1 0];
LLinkage.Joint = LLinkageJoint;

RLinkage = robotics.RigidBody('RLinkage');
RLinkageJoint = robotics.Joint('RLinkageJoint', 'fixed');
setFixedTransform(RLinkageJoint,trvec2tform([linkageLength 0 0]));
RLinkage.Joint = RLinkageJoint;

ULinkage = robotics.RigidBody('ULinkage');
ULinkageJoint = robotics.Joint('ULinkageJoint', 'revolute');
ULinkageJoint.HomePosition = pi/4;
setFixedTransform(ULinkageJoint,trvec2tform([0 linkageLength 0]));
ULinkageJoint.JointAxis = [1 0 0];
ULinkage.Joint = ULinkageJoint;

DLinkage = robotics.RigidBody('DLinkage');
DLinkageJoint = robotics.Joint('DLinkageJoint', 'fixed');
setFixedTransform(DLinkageJoint,trvec2tform([0 linkageLength 0]));
DLinkage.Joint = DLinkageJoint;

addBody(robot,LLinkage, 'LServoArm');
addBody(robot,RLinkage, 'RServoArm');
addBody(robot,ULinkage, 'UServoArm');
addBody(robot,DLinkage, 'DServoArm');
%% 
%Add secondary-axis revolute joints as the "ball and socket"
% Left ball and socket
LBallAndSocket = robotics.RigidBody('LBallAndSocket');
LBallAndSocketJoint = robotics.Joint('LBallAndSocketJoint', 'revolute');
LBallAndSocketJoint.HomePosition = pi/4;
setFixedTransform(LBallAndSocketJoint,trvec2tform([0 0 0]));
LBallAndSocketJoint.JointAxis = [1 0 0];
LBallAndSocket.Joint = LBallAndSocketJoint;

UBallAndSocket = robotics.RigidBody('UBallAndSocket');
UBallAndSocketJoint = robotics.Joint('UBallAndSocketJoint', 'revolute');
UBallAndSocketJoint.HomePosition = pi/4;
setFixedTransform(UBallAndSocketJoint,trvec2tform([0 0 0]));
UBallAndSocketJoint.JointAxis = [0 1 0];
UBallAndSocket.Joint = UBallAndSocketJoint;


addBody(robot,LBallAndSocket, 'LLinkage');
addBody(robot,UBallAndSocket, 'ULinkage');

%Add third yaw revolute joints as the "ball and socket yaw"
LBallAndSocketYaw = robotics.RigidBody('LBallAndSocketYaw');
LBallAndSocketYawJoint = robotics.Joint('LBallAndSocketYawJoint', 'revolute');
LBallAndSocketYawJoint.HomePosition = pi/4;
setFixedTransform(LBallAndSocketYawJoint,trvec2tform([0 0 0]));
LBallAndSocketYawJoint.JointAxis = [0 0 1];
LBallAndSocketYaw.Joint = LBallAndSocketYawJoint;

UBallAndSocketYaw = robotics.RigidBody('UBallAndSocketYaw');
UBallAndSocketYawJoint = robotics.Joint('UBallAndSocketYawJoint', 'revolute');
UBallAndSocketYawJoint.HomePosition = pi/4;
setFixedTransform(UBallAndSocketYawJoint,trvec2tform([0 0 0]));
UBallAndSocketYawJoint.JointAxis = [0 0 1];
UBallAndSocketYaw.Joint = UBallAndSocketYawJoint;


addBody(robot,LBallAndSocketYaw, 'LBallAndSocket');
addBody(robot,UBallAndSocketYaw, 'UBallAndSocket');
%% 
%End effector

LEndEffector = robotics.RigidBody('LEndEffector');
LEndEffectorJoint = robotics.Joint('LEndEffectorJoint', 'fixed');
setFixedTransform(LEndEffectorJoint,trvec2tform([swashPlateLength/2 0 0]));
LEndEffector.Joint = LEndEffectorJoint;

UEndEffector = robotics.RigidBody('UEndEffector');
UEndEffectorJoint = robotics.Joint('UEndEffectorJoint', 'fixed');
setFixedTransform(UEndEffectorJoint,trvec2tform([0 swashPlateLength/2 0]));
UEndEffector.Joint = UEndEffectorJoint;


addBody(robot,LEndEffector, 'LBallAndSocketYaw');
addBody(robot,UEndEffector, 'UBallAndSocketYaw');

%End effector

LEndEffector2 = robotics.RigidBody('LEndEffector2');
LEndEffectorJoint2 = robotics.Joint('LEndEffectorJoint2', 'fixed');
setFixedTransform(LEndEffectorJoint2,trvec2tform([swashPlateLength/2 0 0]));
LEndEffector2.Joint = LEndEffectorJoint2;

UEndEffector2 = robotics.RigidBody('UEndEffector2');
UEndEffectorJoint2 = robotics.Joint('UEndEffectorJoint2', 'fixed');
setFixedTransform(UEndEffectorJoint2,trvec2tform([0 swashPlateLength/2 0]));
UEndEffector2.Joint = UEndEffectorJoint2;

endEffector = robotics.RigidBody('endEffector');
endEffectorJoint = robotics.Joint('endEffectorJoint', 'fixed');
setFixedTransform(endEffectorJoint,trvec2tform([0 0 3]));
endEffector.Joint = endEffectorJoint;

UDEndEffector = robotics.RigidBody('UDEndEffector');
UDEndEffectorJoint = robotics.Joint('UDEndEffectorJoint', 'fixed');
setFixedTransform(UDEndEffectorJoint,trvec2tform([0 0 -3]));
UDEndEffector.Joint = UDEndEffectorJoint;

addBody(robot,LEndEffector2, 'LEndEffector');
addBody(robot,UEndEffector2, 'UEndEffector');
addBody(robot,endEffector,'LEndEffector');
addBody(robot,UDEndEffector,'UEndEffector');

%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)

figure
show(robot);
view(2)
%% 
%Inverse Kinematics
positionTolerance = 1e-6;

gik = robotics.GeneralizedInverseKinematics('RigidBodyTree',robot);

gik.ConstraintInputs = {'position',...
                        'position',...
                        'position',...
                        'position',...
                        'orientation',...
                        'position'

                        };       % Joint limits
                      
gik.SolverParameters.AllowRandomRestart = false;

%Position Constraints for linkages to end effectors
positionTargetLM = robotics.PositionTarget('LEndEffector','ReferenceBody','middle');
positionTargetLM.TargetPosition = [0 0 0];
positionTargetLM.Weights = 50;
positionTargetLM.PositionTolerance = positionTolerance;

positionTargetLR = robotics.PositionTarget('LEndEffector2','ReferenceBody','RLinkage');
positionTargetLR.TargetPosition = [0 0 0];
positionTargetLR.Weights = 50;
positionTargetLR.PositionTolerance = positionTolerance;

positionTargetUM = robotics.PositionTarget('UEndEffector','ReferenceBody','middle');
positionTargetUM.TargetPosition = [0 0 0];
positionTargetUM.Weights = 50;
positionTargetUM.PositionTolerance = positionTolerance;

positionTargetUD = robotics.PositionTarget('UEndEffector2','ReferenceBody','DLinkage');
positionTargetUD.TargetPosition = [0 0 0];
positionTargetUD.Weights = 50;
positionTargetUD.PositionTolerance = positionTolerance;

positionTargetEndEffectors = robotics.PositionTarget('UDEndEffector','ReferenceBody','endEffector');
positionTargetEndEffectors.TargetPosition = [0 0 0];
positionTargetEndEffectors.Weights = 25;
positionTargetEndEffectors.PositionTolerance = positionTolerance;

finOrientation = robotics.OrientationTarget('endEffector');
finOrientation.OrientationTolerance = deg2rad(0);
finOrientation.Weights = 50;

iniGuess = [0.0630393136777788;0.0630400171487887;0.0630382799224363;0.0630378418430514;1.38218075848645;1.75941103621286;1.75941320812916;1.38217981926081;1.31914145102465;1.31914099759299;1.50379994457534e-08;-3.06484264313506e-07;2.32693969055182e-09;3.06375857311355e-07];

%% 
%Solve Inverse Kinematics for Single Point
roll = 20;
pitch = 10;
yaw = 0;
finOrientation.TargetOrientation = eul2quat([deg2rad(yaw) deg2rad(pitch) deg2rad(roll)]);
[solutions, solutionInfo] = gik(iniGuess, positionTargetLM, positionTargetLR, positionTargetUM, positionTargetUD, finOrientation, positionTargetEndEffectors);

ViolationLM = solutionInfo.ConstraintViolations(1).Violation;
ViolationLR = solutionInfo.ConstraintViolations(2).Violation;
ViolationUM = solutionInfo.ConstraintViolations(3).Violation;
ViolationUD = solutionInfo.ConstraintViolations(4).Violation;
ViolationLRFinOrientation = solutionInfo.ConstraintViolations(5).Violation;
ViolationUDFinOrientation = solutionInfo.ConstraintViolations(6).Violation;

show(robot,solutions,'PreservePlot',false);
xlim([-2 2]);
ylim([-2 2]);
zlim([-1 5]);
view([1 1 1]);
drawnow

output = [roll pitch rad2deg(solutions(1:4,1))' ViolationLM ViolationLR ViolationUM ViolationUD ViolationLRFinOrientation ViolationUDFinOrientation]

% %% 
% %Solve inverse kinematics for trajectory
% datas = csvread("trajectorydata\trajectories\Gen_0_C_0_MaxAng_19.1_ThkAng_6.8_RotAng_11.9_RotInt_1.4_SpdCde_3_Spdupv_0.3_Kv_0.5_hdev_0.9_freq_0.18_TB.csv");
% size = length(datas);
% output = zeros(size(1),12);
% 
% pitch_datas = datas(:, 3);
% roll_datas = datas(:, 4);
% 
% 
%     
% for idx = 1:1:size(1); 
%     roll = roll_datas(idx);
%     pitch = pitch_datas(idx);
%     yaw = 0;
% 
%     finOrientation.TargetOrientation = eul2quat([deg2rad(yaw) deg2rad(roll) deg2rad(pitch)]);
%     [solutions, solutionInfo] = gik(iniGuess, positionTargetLM, positionTargetLR, positionTargetUM, positionTargetUD, finOrientation, positionTargetEndEffectors);
%     
%     ViolationLM = solutionInfo.ConstraintViolations(1).Violation;
%     ViolationLR = solutionInfo.ConstraintViolations(2).Violation;
%     ViolationUM = solutionInfo.ConstraintViolations(3).Violation;
%     ViolationUD = solutionInfo.ConstraintViolations(4).Violation;
%     ViolationLRFinOrientation = solutionInfo.ConstraintViolations(5).Violation;
%     ViolationUDFinOrientation = solutionInfo.ConstraintViolations(6).Violation;
%     
%     iniGuess = solutions;
% 
%     show(robot,solutions,'PreservePlot',false);
%     xlim([-2 2]);
%     ylim([-2 2]);
%     zlim([-1 5]);
%     view([1 1 1]);
%     drawnow
% 
%     output(idx,:) = [roll pitch rad2deg(solutions(1:4,1))' ViolationLM ViolationLR ViolationUM ViolationUD ViolationLRFinOrientation ViolationUDFinOrientation]
% end
%% 