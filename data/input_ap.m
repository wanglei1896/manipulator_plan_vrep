%% along path规划的输入数据
% 包含机械臂起点和终点的末端位置（齐次矩阵）
% 输入： 
%    path: 路径的点序列[xseq; 
%                      yseq; 
%                      zseq] 
function result = input_ap()
    result.task_name = 'test'; %要完成的任务的名称，具体任务由vrep端配置
    result.pStart = [];
    result.qStart = [];
    result.spacenum = 0;
    result.path = [];
    result.obstacles = [];
    result.obstacle_num = 0;
end

