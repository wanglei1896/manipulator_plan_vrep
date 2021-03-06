%% along path规划的优化日志
function result = optimLog_ap(group_num)
    result.group_num = group_num;
    for i=1:result.group_num
        result.group(i).fitness_history = [];
        result.group(i).solution_history = [];
        result.group(i).fitvec_history = [];
        result.group(i).all_solution_history = [];
        result.group(i).all_fitness_history = [];
    end
    result.progress=0; %用于记录进度
    result.sum.fitness_history = [];
    result.qTable_history = [];
    result.jointPath_history = [];
    result.workPath_history = [];
end

