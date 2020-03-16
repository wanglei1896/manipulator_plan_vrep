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
inputData.pStart = [1 0 0 0.3;
                    0 1 0 0.4;
                    0 0 1 0;
                    0 0 0 1];
inputData.pFinal = [1 0 0 0.6;
                    0 1 0 0;
                    0 0 1 0;
                    0 0 0 1];
[inputData.qStart, inputData.qFinal] = ...
    regular_JointPos(model.ikunc(inputData.pStart), model.ikunc(inputData.pFinal));
outputData.spacenum = 20;