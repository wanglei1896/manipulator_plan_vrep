% 主流程控制
%{
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model           %机械臂的运动学模型
%}
%% 配置、环境数据获取
initial_native_p2p;
%%% 生成
%initial_fromVrep;

%% 规划过程
planner_p2p;

%% 在环境中执行规划出的位姿/轨迹
%executeInVrep;
executeInMatlab;
