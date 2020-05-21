% 主流程控制

global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model ...         %机械臂的运动学模型

%% 运行环境
ProjectPath = fileparts(mfilename('fullpath'));
addpath(fullfile(ProjectPath, 'initial')); % 此文件夹下放规划的初始化脚本
addpath(fullfile(ProjectPath, 'planner')); % 放规划的脚本
addpath(fullfile(ProjectPath, 'execute')); % ִ放执行\展示规划结果的脚本
addpath(fullfile(ProjectPath, 'analyze')); % 分析核心优化函数的优化过程
addpath(fullfile(ProjectPath, 'data')); % 公共数据的'构造'函数
addpath(fullfile(ProjectPath, 'modular')); % 存放核心优化算法、代价函数、以及各种工具函数
addpath(fullfile(ProjectPath, 'rvctools')); % 机器人库
addpath(fullfile(ProjectPath, 'connectionTool')); %与vrep进行通信的库
startup_rvc; % 配置机器人库环境

%% 配置、环境数据获取
initial_ap;

%% 规划过程(第一阶段)
% 超参数汇总：
hyperparameter.ap1_delta=0.1; %控制相邻规划点上各关节相较上一个规划点的活动范围
hyperparameter.ap1_to1=0; %fq与cost的混合比例，tradeoff
hyperparameter.ap1_obflag=false; %是否开启避障(耗时)
hyperparameter.ap1_to2=1/3; %避障部分与fdt代价的混合比例，tradeoff
hyperparameter.ob_e=1e-3; %避障部分的最小距离，低于此最小距离则代价不再增长
hyperparameter.ob_beta=1; %避障部分代价函数中的指数系数
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);
outputData.jointPath=planner_ap1(model, inputData, hyperparameter);

%% 规划过程(第二阶段)
% 超参数汇总：
hyperparameter.ap2_tradeoff=[1 1 1 1 1 1]; %代价函数中各指标的混合比例
hyperparameter.ap2_obflag=false; %是否开启避障(耗时)
% 清空optimLog
optimLog = optimLog_ap(optimLog.group_num); %优化有几个组
outputData=planner_ap2(model, outputData.jointPath, inputData.obstacles, hyperparameter);

%% 在环境中执行规划出的位姿/轨迹
executeInVrep;
% executeInMatlab;
