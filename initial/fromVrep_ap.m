disp('Sample from V-rep');
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19998,true,true,5000,5);

assert(clientID>-1);
if (clientID>-1)
    disp('Connected to remote API server');
    % main
    fromVrep(vrep, clientID);
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

function fromVrep(vrep, clientID)
global inputData
    try
        vrep.simxSetIntegerSignal(clientID,'SIG_start',1,vrep.simx_opmode_oneshot);
        vrep.simxSetIntegerSignal(clientID,'SIG_spacenum',inputData.spacenum,vrep.simx_opmode_oneshot);
        while true  %等待vrep端采样结束
            [res, signal]=vrep.simxGetIntegerSignal(clientID,'SIG_finish',vrep.simx_opmode_oneshot_wait);
            if signal
                vrep.simxClearIntegerSignal(clientID,'SIG_finish',vrep.simx_opmode_oneshot);
                break;
            end
        end
        [res, signal]=vrep.simxGetStringSignal(clientID,'Data_targetPath',vrep.simx_opmode_oneshot_wait);
        path = reshape(vrep.simxUnpackFloats(signal),3,inputData.spacenum+1);
        % disp('sampled path:')
        % disp(path)
        disp('sampling complete')
        inputData.path = path;
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(['error in "', e.stack(1).name, '": line ', num2str(e.stack(1).line)])
        disp(e.message)
    end
end