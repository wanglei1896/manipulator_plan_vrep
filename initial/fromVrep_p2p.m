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
global inputData model
    try
        % get handle
        for i=1:6
            [res,handle_rigArmjoint(i)] = vrep.simxGetObjectHandle(clientID,['UR5_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
            assert(res==0);
        end
        [res,handle_pStart] = vrep.simxGetObjectHandle(clientID,'pStart',vrep.simx_opmode_oneshot_wait);
        assert(res==0);
        [res,handle_pFinal] = vrep.simxGetObjectHandle(clientID,'pFinal',vrep.simx_opmode_oneshot_wait);
        assert(res==0);
        n_obstacle=1;
        while true
            [res, ohandle] = vrep.simxGetObjectHandle(clientID,['obstacle_',num2str(n_obstacle)],vrep.simx_opmode_oneshot_wait);
            %disp(ohandle)
            if res~=0  %不知道有多少个障碍物，按编号1开始get，直到get不到为止
                break;
            end
            n_obstacle=n_obstacle+1;
        end
        inputData.obstacle_num=n_obstacle-1;
        disp([num2str(inputData.obstacle_num),' obstacles in total.'])
        
        % get vrep data
        [returnCode,pStart_pos]=vrep.simxGetObjectPosition(clientID,handle_pStart,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
        [returnCode,pStart_rot]=vrep.simxGetObjectOrientation(clientID,handle_pStart,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
        [returnCode,pFinal_pos]=vrep.simxGetObjectPosition(clientID,handle_pFinal,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
        [returnCode,pFinal_rot]=vrep.simxGetObjectOrientation(clientID,handle_pFinal,handle_rigArmjoint(1),vrep.simx_opmode_blocking);
        inputData.pStart = double(transl(pStart_pos(1),pStart_pos(2),pStart_pos(3))...
            *trotx(pStart_rot(1))*troty(pStart_rot(2))*trotz(pStart_rot(3)));
        inputData.pFinal = double(transl(pFinal_pos(1),pFinal_pos(2),pFinal_pos(3))...
            *trotx(pFinal_rot(1))*troty(pFinal_rot(2))*trotz(pFinal_rot(3)));
        vrep.simxSetIntegerSignal(clientID,'getVex_start',1,vrep.simx_opmode_oneshot);
        while true  %等待vrep端采样结束
            [res, signal]=vrep.simxGetIntegerSignal(clientID,'getVex_finish',vrep.simx_opmode_oneshot_wait);
            if signal
                vrep.simxClearIntegerSignal(clientID,'getVex_finish',vrep.simx_opmode_oneshot);
                break;
            end
        end
        [res, raw_vex]=vrep.simxGetStringSignal(clientID,'Data_Verteces',vrep.simx_opmode_oneshot_wait);
        [res, raw_face]=vrep.simxGetStringSignal(clientID,'Data_Faces',vrep.simx_opmode_oneshot_wait);
        
        vex=double(vrep.simxUnpackFloats(raw_vex));
        faces=int32(vrep.simxUnpackInts(raw_face));
        for i=1:inputData.obstacle_num
            numVex=int32(vex(1));
            numFace=faces(1);
            inputData.obstacles(i).vex=reshape(vex(2:numVex+1),3,numVex/3);
            inputData.obstacles(i).face=reshape(faces(2:numFace+1),3,numFace/3);
            inputData.obstacles(i).vn=numVex/3;
            vex=vex(numVex+2:end);
            faces=faces(numFace+2:end);
        end
        total_link_vn=0;
        for i=1:6
            numVex=int32(vex(1));
            numFace=faces(1);
            model.shape(i).vex=reshape(vex(2:numVex+1),3,numVex/3);
            model.shape(i).face=reshape(faces(2:numFace+1),3,numFace/3);
            model.shape(i).vn=numVex/3;
            total_link_vn=total_link_vn+numVex/3;
            vex=vex(numVex+2:end);
            faces=faces(numFace+2:end);
        end
        model.num_shapeVertex=total_link_vn;
        
        disp('sampling ended')
        %disp(['Initial Joint is [',num2str(initialJoint),']']);
        disp('Start Point is ')
        disp(inputData.pStart)
        disp('Final Point is ')
        disp(inputData.pFinal)
    catch e
        vrep.simxPauseCommunication(clientID,0);
        disp(['error in "', e.stack(1).name, '": line ', num2str(e.stack(1).line)])
        disp(e.message)
    end
end