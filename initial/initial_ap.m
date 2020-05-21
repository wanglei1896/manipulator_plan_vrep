%% 在matlab本地初始化各along path规划数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model ...         %机械臂的运动学模型
        vrep %与vrep的通信工具
isTest=false;
isFromVrep=true;

%%% 初始化
model = model_serialLink();
optimLog = optimLog_ap(10);   %优化有几个组
inputData = input_ap([[0.3;0.4;0], [0.3;0.4;0]+1*[0.3;-0.4;0]],... %输入的路径(默认)
                    optimLog.group_num*5);   %输入路径规范化后的采样段数
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
