%% 在matlab本地初始化各数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model           %机械臂的运动学模型

%%% 初始化
model = model_ur5();
optimLog = optimLog_p2p();
inputData = input_p2p();
outputData = output_multiSeg();

%%% 默认
inputData.pStart = [1 0 0 0.8;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];
inputData.pFinal = [1 0 0 0.2;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];
outputData.spacenum = 100;

% 从vrep中读入数据(pStart, pFinal)
% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
fromVrep_p2p;

[inputData.qStart, inputData.qFinal] = ...
    regular_JointPos(model.km.ikine(inputData.pStart,'rdf'), model.km.ikine(inputData.pFinal,'rdf'));