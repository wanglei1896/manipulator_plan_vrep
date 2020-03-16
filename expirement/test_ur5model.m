% 用于测试matlab工具箱中模型与v-rep模型的匹配度
% @require 
%   outputjointValue：   输入给vrep的关节运动角
%   diseredPosition：    matlab toolbox坐标系下的tip坐标
% @
%   actualPosition：     vrep坐标系下运动的tip坐标

global outputjointValue diseredPosition

% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    try
        disp('Connected to remote API server');
        % get handle
        for i=1:6
            [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,['UR5_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
        end
        [res,target_dummy] = vrep.simxGetObjectHandle(clientID,'target',vrep.simx_opmode_oneshot_wait);
        [res,tip_dummy] = vrep.simxGetObjectHandle(clientID,'tip',vrep.simx_opmode_oneshot_wait);
        
        while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
            for i=1:length(diseredPosition)
                % Set the position of every joint
                vrep.simxPauseCommunication(clientID,1);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(1),outputjointValue(1,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(2),outputjointValue(2,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(3),outputjointValue(3,i),vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(4),outputjointValue(4,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(5),outputjointValue(5,i),vrep.simx_opmode_oneshot);
                vrep.simxSetJointPosition(clientID,handle_rigArmjoint(6),outputjointValue(6,i)+pi/2,vrep.simx_opmode_oneshot);
                vrep.simxPauseCommunication(clientID,0);
                % 用target_dummy代表toolbox坐标系下的路径
                vrep.simxSetObjectPosition(clientID,target_dummy,handle_rigArmjoint(1),diseredPosition(:,i),vrep.simx_opmode_oneshot);
                % vrep端的路径
                [returnCode,actualPosition(:,i)]=vrep.simxGetObjectPosition(clientID,tip_dummy,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
                
                pause(0.1);
            end
            break;
        end
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(e);
    end
    vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');