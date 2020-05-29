global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model ...         %机械臂的运动学模型
        hyperparameter

%% 规划过程(第一阶段)
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);
outputData.jointPath=planner_ap1(model, inputData, hyperparameter);

%% 规划过程(第二阶段)
% 清空optimLog
optimLog = optimLog_ap(optimLog.group_num); %优化有几个组
outputData=planner_ap2(model, outputData.jointPath, inputData.obstacles, hyperparameter);