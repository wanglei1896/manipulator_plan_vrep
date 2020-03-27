% 主流程控制

%{
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        model           %机械臂的运动学模型
%}

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
% initial_p2p

%% 规划过程
planner_ap;
% planner_p2p;

%% 在环境中执行规划出的位姿/轨迹
executeInVrep;
% executeInMatlab;
