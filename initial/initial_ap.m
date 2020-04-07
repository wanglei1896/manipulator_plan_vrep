%% 在matlab本地初始化各along path规划数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model           %机械臂的运动学模型

%%% 初始化
model = model_ur5();
optimLog = optimLog_ap(2);   %优化有几个组
inputData.spacenum = optimLog.group_num*10;  %输入路径规范化后的采样段数
inputData = input_ap([0.3, 0.6;    %输入的路径(默认)
                      0.4,   0;
                        0,   0]); 
% 元循环，将上次优化的结果路径作为本次的目标路径
isTest=false;
if ~isequal(outputData,[]) && ~isequal(outputData.endPath,[]) && isTest
    inputData=input_ap(outputData.endPath,20);
end
outputData = output_multiSeg();

%%% 默认
outputData.spacenum = optimLog.group_num*10;

%%% 从vrep中读取数据
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
fromVrep_ap;

inputData.pStart = eye(4);
inputData.pStart(1:3,4) = inputData.path(:,1);
inputData.qStart = model.ikunc(inputData.pStart);