%% 运行环境
start_up;

%% 配置、环境数据获取
initial_ap;
hyperparameter.ap1_obflag=false;
hyperparameter.ap2_obflag=false;

%% 规划过程
planner_ap;

%% 在环境中执行规划出的位姿/轨迹
executeInVrep;
% executeInMatlab;
