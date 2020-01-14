global outputjointValue diseredPosition actualPosition
%clear vrep clinetID
disp('Send to V-rep');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

%read the joint angle data from 'angle.txt'
%load model\theta.mat; %A matrix of 7 x 150.Each column vector recorded the changes of each joint Angle 
%jointValue = theta'; 
[m n]=size(outputjointValue);

if (clientID>-1)
try
disp('Connected to remote API server');
% get handle
for i=1:6
    [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,['UR5_joint',num2str(i)],vrep.simx_opmode_oneshot_wait); 
end
[res,target_dummy] = vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_oneshot_wait);
[res,tip_dummy] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_oneshot_wait);

%Set the position of every joint
while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
for i=1:length(diseredPosition)
vrep.simxPauseCommunication(clientID,1); 
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(1),outputjointValue(1,i)+pi/2,vrep.simx_opmode_oneshot); 
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(2),outputjointValue(2,i)+pi/2,vrep.simx_opmode_oneshot); 
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(3),outputjointValue(3,i),vrep.simx_opmode_oneshot); 
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(4),outputjointValue(4,i)+pi/2,vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(5),outputjointValue(5,i),vrep.simx_opmode_oneshot);
vrep.simxSetJointPosition(clientID,handle_rigArmjoint(6),outputjointValue(6,i)+pi/2,vrep.simx_opmode_oneshot);
vrep.simxPauseCommunication(clientID,0);

vrep.simxSetObjectPosition(clientID,target_dummy,handle_rigArmjoint(1),diseredPosition(:,i),vrep.simx_opmode_oneshot);
[returnCode,actualPosition(:,i)]=vrep.simxGetObjectPosition(clientID,tip_dummy,handle_rigArmjoint(1),vrep.simx_opmode_blocking);

pause(0.1);
end 
break;
end
catch e
    vrep.simxPauseCommunication(clientID,0);
    disp(e);
end
% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);
% Now close the connection to V-REP:
vrep.simxFinish(clientID);
else
disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');