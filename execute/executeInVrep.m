% 规划生成的数据(outputData)在vrep中执行，并从vrep中收集一些数据(fromVrepData)

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
global outputData model vrep fromVrepData
%     diseredPath=inputData.path;
    joint_angle=outputData.trajectory(1:model.joint_num,:);
    %joint_angle=outputData.jointPath;
    try
        % get handle
        [res,handle_vision] = vrep.simxGetObjectHandle(clientID,...
                'Vision_sensor',vrep.simx_opmode_oneshot_wait);
        handle_rigArmjoint=zeros(1,model.joint_num);
        for i=1:model.joint_num
            [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,...
                [model.name,'_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
        end
        
        %Set the position of every joint
        
        while(vrep.simxGetConnectionId(clientID) ~= -1) % while v-rep connection is still active
            snapshots=[];
            snapshots=uint8(snapshots);
            [~,resolution,image]=vrep.simxGetVisionSensorImage2(...
                clientID,handle_vision,0,vrep.simx_opmode_streaming);
            snapshot_count=0;
            vrep.simxSetIntegerSignal(clientID,'SIG_execute',1,vrep.simx_opmode_oneshot);
            for i=1:size(joint_angle,2)
                vrep.simxPauseCommunication(clientID,1);
                for j=1:model.joint_num
                    ret=vrep.simxSetJointPosition(clientID,handle_rigArmjoint(j),joint_angle(j,i),vrep.simx_opmode_oneshot);
                    %disp(ret)
                end
                vrep.simxPauseCommunication(clientID,0);
                
                [returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(...
                    clientID,handle_vision,0,vrep.simx_opmode_buffer);
                if ~isempty(image)
                    snapshot_count=snapshot_count+1;
                    snapshots(:,:,:,snapshot_count)=image;
                end
                % 为了有充分时间截图，这里等待时间长一点
                pause(1);
                
            end
            
            vrep.simxSetIntegerSignal(clientID,'SIG_execute_end',1,vrep.simx_opmode_oneshot);
            while true
                [res, signal]=vrep.simxGetIntegerSignal(clientID,'SIG_send_distance',vrep.simx_opmode_oneshot_wait);
                if signal==1
                    [res, raw_distance]=vrep.simxGetStringSignal(clientID,'Data_distance',vrep.simx_opmode_oneshot_wait);
                    vrep.simxSetIntegerSignal(clientID,'SIG_send_distance',0,vrep.simx_opmode_oneshot);
                    break;
                end
            end
            %{
            pause(0.1);
            vrep.simxSetIntegerSignal(clientID,'SIG_send_sanpshot',1,vrep.simx_opmode_oneshot);
            snapshot_count=0;
            while true
                [res, send_sanpshot]=vrep.simxGetIntegerSignal(clientID,'SIG_send_sanpshot',vrep.simx_opmode_oneshot_wait);
                if send_sanpshot==2
                    break;
                end
                snapshot_count=snapshot_count+1;
                [res, raw_snapshot]=vrep.simxGetStringSignal(clientID,'Data_snapshots',vrep.simx_opmode_oneshot_wait);
                snapshots(snapshot_count,:)=double(vrep.simxUnpackFloats(raw_snapshot));
            end
            fromVrepData.snapshot=snapshots;
            %}
            break;
        end
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(['error in "', e.stack(1).name, '": line ', num2str(e.stack(1).line)])
        disp(e.message)
    end
    fromVrepData.snapshot=snapshots;
    fromVrepData.mindis_variation=double(vrep.simxUnpackFloats(raw_distance));
    %vrep.simxSetIntegerSignal(clientID,'SIG_execute',0,vrep.simx_opmode_oneshot);
end