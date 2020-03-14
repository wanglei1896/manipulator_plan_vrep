%% 点到点规划，机械臂的输出位姿轨迹（关节角）

function [result] = output_p2p()
    result.spacenum = 0;   %轨迹采样段数
    result.trajectory = []; %轨迹数据
    result.middle_time = 0;
    result.total_time = 0;
end

