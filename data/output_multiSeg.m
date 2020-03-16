%% along path规划，机械臂的输出位姿轨迹（关节角）

function [result] = output_multiSeg()
    result.spacenum = 0;   %轨迹采样段数
    result.trajectory = []; %轨迹数据
    result.segment_times = []; %每段轨迹的时间间隔
    result.segment_curtimes = []; %累积的时间间隔(时刻)
end

