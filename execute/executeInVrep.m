%clear vrep clinetID
disp('Send to V-rep');
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    % main sender
    send2vrep(vrep, clientID);
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!
disp('Program ended');

function send2vrep(vrep, clientID)
global inputData outputData
    diseredPath=inputData.path;
    joint_angle=outputData.trajectory;
    try
        % get handle
        for i=1:6
            [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,...
                ['UR5_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
        end
        [res,target_dummy] = vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_oneshot_wait);
        [res,tip_dummy] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_oneshot_wait);
        
        %Set the position of every joint
        while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
            for i=1:inputData.spacenum+1
                vrep.simxSetObjectPosition(clientID,target_dummy,handle_rigArmjoint(1),diseredPath(:,i),vrep.simx_opmode_oneshot);
            end
            for i=1:outputData.spacenum+1
                vrep.simxPauseCommunication(clientID,1);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(1),joint_angle(1,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(2),joint_angle(2,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(3),joint_angle(3,i),vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(4),joint_angle(4,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(5),joint_angle(5,i),vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(6),joint_angle(6,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxPauseCommunication(clientID,0);

                %[returnCode,actualPosition(:,i)]=vrep.simxGetObjectPosition(clientID,tip_dummy,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
                pause(0.1);
            end
            break;
        end
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(e);
    end
end