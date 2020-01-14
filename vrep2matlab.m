global initialJoint pStart pFinal
disp('Sample from V-rep');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19998,true,true,5000,5);

if (clientID>-1)
disp('Connected to remote API server');
% get handle
for i=1:6
    [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,['UR5_joint',num2str(i)],vrep.simx_opmode_oneshot_wait); 
end
[res,handle_pStart] = vrep.simxGetObjectHandle(clientID,'pStart',vrep.simx_opmode_oneshot_wait);
[res,handle_pFinal] = vrep.simxGetObjectHandle(clientID,'pFinal',vrep.simx_opmode_oneshot_wait);

%Set the position of every joint
while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
for i=1:6
    [returnCode,initialJoint(i)]=vrep.simxGetJointPosition(clientID,handle_rigArmjoint(i),vrep.simx_opmode_blocking);
end
[returnCode,pStart_pos]=vrep.simxGetObjectPosition(clientID,handle_pStart,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
[returnCode,pStart_rot]=vrep.simxGetObjectOrientation(clientID,handle_pStart,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
[returnCode,pFinal_pos]=vrep.simxGetObjectPosition(clientID,handle_pFinal,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
[returnCode,pFinal_rot]=vrep.simxGetObjectOrientation(clientID,handle_pFinal,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
break;
end
% Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID);
% Now close the connection to V-REP:
vrep.simxFinish(clientID);
else
disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

pStart = transl(pStart_pos(1),pStart_pos(2),pStart_pos(3))...
    *trotx(pStart_rot(1))*troty(pStart_rot(2))*trotz(pStart_rot(3));
pFinal = transl(pFinal_pos(1),pFinal_pos(2),pFinal_pos(3))...
    *trotx(pFinal_rot(1))*troty(pFinal_rot(2))*trotz(pFinal_rot(3));
disp('sampling ended');
disp(['Initial Joint is [',num2str(initialJoint),']']);
disp('Start Point is ');
disp(pStart)
disp('Final Point is ');
disp(pFinal);