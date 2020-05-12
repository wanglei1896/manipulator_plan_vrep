% 原始bas算法，主要用于ap规划的第一阶段

% 返回的fitness_history与solution_history都是每次迭代中的种群最优个体数据
% fitvec_history是多目标优化时各指标分量向量
function [fitness_history, fitvec_history, solution_history,optimization_time]...
    = AlgorithmBAS_fun(iteration_num,solution_bound,fitnessfun)
    %% properties
    obj_iteration_num = iteration_num;
    obj_solution_bound = solution_bound;
    %obj_precision=0.001;
    obj_decay_rate=0.99; %衰减率
    obj_ratio=2; %ratio of antenna_length and step_length
    obj_step_length=0.5; %天牛的步长
    obj_ante_dis=obj_step_length*obj_ratio; %天牛的探测范围直径(触须长)
    
    fitness_history = zeros(obj_iteration_num, 1);
    fitvec_history = [];
    solution_history = zeros(obj_iteration_num, obj_dimension);
    % 生成初始天牛位置
    sol=rand(1,obj_dimension);
    %% main procedure
    tic
    % 迭代寻优
    for ii=1:obj_iteration_num
        updateSol();
        %updateFintness
        [obj_fitness, obj_fitvec] = fitnessfun(restore_bound(sol));
        %步长衰减
        obj_step_length = obj_step_length * obj_decay_rate;
        obj_ante_dis = obj_step_length*obj_ratio;
        % 存储当前迭代最优结果
        fitness_history(ii)=1/obj_fitness;
        fitvec_history(ii,:)=obj_fitvec;
        solution_history(ii,:)=restore_bound(sol);
    end
    toc
    optimization_time=toc;
    
    %% subfunctions
    function dimension = obj_dimension()
        dimension = size(obj_solution_bound,1);
    end
    function updateSol()
        dir = rands(1,obj_dimension);
        dir = dir/(eps+norm(dir));
        xleft=handle_border(sol+dir*(obj_ante_dis/2));
        fleft=fitnessfun(restore_bound(xleft));
        xright=handle_border(sol-dir*(obj_ante_dis/2));
        fright=fitnessfun(restore_bound(xright));
        % 位置更新
        sol=handle_border(sol+obj_step_length*dir*sign(fleft-fright));
    end
    %边界处理
    function outputvec = handle_border(inputvec)
        outputvec = inputvec;
        max_i=inputvec>1;
        outputvec(max_i) = 1;
        min_i=inputvec<0;
        outputvec(min_i) = 0;
    end
    %算法内部各维度的寻优范围都是[0,1],计算适应度值时映射到给定范围
    function result = restore_bound(solution)
        for s=solution
           assert(s>=0 && s<=1)
        end
        lower = obj_solution_bound(:,1)';
        upper = obj_solution_bound(:,2)';
        result = (upper-lower).*solution+lower;
        assert(isequal(size(result),[1,obj_dimension]))
    end
end
