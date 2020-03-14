%% along path规划的优化日志
function result = optimLog_ap(group_num)
    result.group_num = group_num;
    for i=1:result.group_num
        result.group(i).fitness_history = [];
        result.group(i).solution_history = [];
    end
    result.sum.fitness_history = [];
    result.sum.solution_history = [];
end

