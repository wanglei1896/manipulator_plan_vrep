%% 在matlab本地初始化各along path规划数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model ...         %机械臂的运动学模型
        vrep ...        %与vrep的通信工具
        hyperparameter   %待调参数
    
isTest=false;
isFromVrep=true;

%%% 初始化
hyperparameter = hyperparameter_ap();
model = model_serialLink();
optimLog = optimLog_ap(6);   %优化有几个组
inputData = input_ap();
inputData.spacenum=optimLog.group_num*5;
% isTest开启时元循环，将上次优化的结果路径作为本次的目标路径
if ~isequal(outputData,[]) && ~isequal(outputData.endPath,[]) && isTest
    inputData=input_ap(outputData.endPath,20);
end
outputData = output_multiSeg();

%%% 从vrep中读取数据
if isFromVrep
    vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
    fromVrep_ap;
end

inputData.pStart = eye(4);
inputData.pStart(1:3,4) = inputData.path(:,1);
inputData.qStart = regular_JointPos(model.km.ikunc(inputData.pStart));
