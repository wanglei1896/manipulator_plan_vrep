global vrep
%clear vrep clinetID
disp('Send to V-rep');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    % main sender
    send2vrep(clientID);
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!
disp('Program ended');
    
function send2vrep(clientID)
global outputData model vrep
%     diseredPath=inputData.path;
    joint_angle=outputData.trajectory;
    %joint_angle=outputData.jointPath;
    try
        % get handle
        for i=1:model.joint_num
            [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,...
                [model.name,'_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
        end
        
        %Set the position of every joint
        while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
            vrep.simxSetIntegerSignal(clientID,'SIG_execute',1,vrep.simx_opmode_oneshot);
            for i=1:size(joint_angle,2)-1
                vrep.simxPauseCommunication(clientID,1);
                for j=1:model.joint_num
                    vrep.simxSetJointPosition(clientID,handle_rigArmjoint(j),joint_angle(j,i),vrep.simx_opmode_oneshot);
                end
                vrep.simxPauseCommunication(clientID,0);

                %[returnCode,actualPosition(:,i)]=vrep.simxGetObjectPosition(clientID,tip_dummy,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
                pause(0.1);
            end
            break;
        end
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(['error in "', e.stack(1).name, '": line ', num2str(e.stack(1).line)])
        disp(e.message)
    end
    vrep.simxSetIntegerSignal(clientID,'SIG_execute',0,vrep.simx_opmode_oneshot);
end