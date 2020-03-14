%% 在matlab本地初始化各along path规划数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model           %机械臂的运动学模型

%%% 初始化
model = model_ur5();
optimLog = optimLog_ap(6);   %优化有几个组
inputData = input_ap([0.2,  0.1;    %输入的路径
                      0.4,  0.3;
                        0, -0.1], optimLog.group_num*10); %输入路径规范化后的采样段数
outputData = output_multiSeg();

%%% 默认
inputData.pStart = [1 0 0 0.2;
                    0 1 0 0.4;
                    0 0 1 0;
                    0 0 0 1];
inputData.qStart = model.ikunc(inputData.pStart);
outputData.spacenum = optimLog.group_num*10;