 vrep=remApi('remoteApi');
 vrep.simxFinish(-1);
 
 clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
 if (clientID>-1)
    disp('Connected to remote API server');
    %handle
    [returnCode,joint1]=vrep.simxGetObjectHandle(clientID,'puma560_joint1',vrep.simx_opmode_blocking); 
    [returnCode,joint2]=vrep.simxGetObjectHandle(clientID,'puma560_joint2',vrep.simx_opmode_blocking); 
    [returnCode,joint3]=vrep.simxGetObjectHandle(clientID,'puma560_joint3',vrep.simx_opmode_blocking); 
    [returnCode,joint4]=vrep.simxGetObjectHandle(clientID,'puma560_joint4',vrep.simx_opmode_blocking); 
    [returnCode,joint5]=vrep.simxGetObjectHandle(clientID,'puma560_joint5',vrep.simx_opmode_blocking); 
    [returnCode,joint6]=vrep.simxGetObjectHandle(clientID,'puma560_joint6',vrep.simx_opmode_blocking); 
    
    %other code
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,0.4,vrep.simx_opmode_oneshot);
    %[returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
    
    for i=1:length(q(:,1))
        %[returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint1,q(i,1),vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint2,q(i,2),vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint3,q(i,3),vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint4,q(i,4),vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint5,q(i,5),vrep.simx_opmode_oneshot);
        [returnCode]=vrep.simxSetJointPosition(clientID,joint6,q(i,6),vrep.simx_opmode_oneshot);

        pause(0.02);
    end
    %[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_motor,0,vrep.simx_opmode_blocking);
    
    vrep.simxFinish(clientID);
 ends
 
 vrep.delete();