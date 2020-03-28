%% along path规划的输入数据
% 包含机械臂起点和终点的末端位置（齐次矩阵）
% 输入： 
%    path: 路径的点序列[xseq; 
%                      yseq; 
%                      zseq] 
function result = input_ap(path, spacenum)
    result.pStart = [];
    result.qStart = [];
    result.spacenum = spacenum;
    result.path = regular_path(path, result.spacenum);
end

