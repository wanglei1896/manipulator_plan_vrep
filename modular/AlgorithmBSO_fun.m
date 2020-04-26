%ALGORITHMBSO_FUN 此处显示有关此函数的摘要
%       AlgorithmBSO的函数封装版本
%       解决回调函数fitnessfun的handle作用域问题

% 返回的fitness_history与solution_history都是每次迭代中的种群最优个体数据
% fitvec_history是多目标优化时各指标分量向量
function [fitness_history, fitvec_history, solution_history,...
    all_solution_history, optimization_time, all_fitness_history]...
    = AlgorithmBSO_fun(sizepop,iteration_num,solution_bound,fitnessfun)
    %% properties
    obj_sizepop = sizepop;
    obj_iteration_num = iteration_num;
    obj_solution_bound = solution_bound;
    %obj_precision=0.001;
    
    obj_decay_rate=0.98; %衰减率
    obj_ratio=3; %ratio of antenna_length and step_length
    obj_step_length=0.2; %天牛的步长
    obj_ante_dis=obj_step_length*obj_ratio; %天牛的探测范围直径(触须长)
    
    %学习因子
    obj_c1=1.49445;
    obj_c2=1.49445;
    %比例因子，决定PSO中的速度与BAS中步长的混合比例
    obj_lamda=0.2;
    %天牛速度限制
    obj_Vmax = 0.2;
    
%     obj_pop;
%     obj_pop_step; %天牛群增量步
%     obj_pop_v; %天牛群原始步(速度) 
%     obj_pop_fitness;
%     obj_gbest; %种群个体极值点
%     obj_gbest_fitness;
%     obj_zbest; %种群全体极值点
%     obj_zbest_fitness;
    
    [obj_pop, obj_pop_step, obj_pop_v] = initialPop();
    [obj_pop_fitness,obj_pop_fitvec, obj_zbest, obj_gbest, obj_gbest_fitness, obj_zbest_fitness]...
        = initialFitness();  
    fitness_history = zeros(obj_iteration_num, 1);
    fitvec_history = [];
    solution_history = zeros(obj_iteration_num, obj_dimension);
    all_solution_history = zeros(obj_iteration_num, obj_sizepop, obj_dimension);
    all_fitness_history = zeros(obj_iteration_num, obj_sizepop);
    %% main procedure
    tic
    % 迭代寻优
    for ii=1:obj_iteration_num
        updatePop();
        updateFitness();
        %步长衰减
        obj_step_length = obj_step_length * obj_decay_rate;
        obj_ante_dis = obj_step_length*obj_ratio;
        % 存储当前迭代最优结果
        [best_fit,best_index]=max(obj_pop_fitness);
        fitness_history(ii)=1/best_fit;
        fitvec_history(ii,:)=obj_pop_fitvec(best_index,:);
        bsol=restore_bound(obj_pop(best_index,:));
        solution_history(ii,:)=bsol;
        all_solution_history(ii,:,:)=obj_pop;
        all_fitness_history(ii,:)=1./obj_pop_fitness;
    end
    toc
    optimization_time=toc;
    
    %% subfunctions
    function dimension = obj_dimension()
        dimension = size(obj_solution_bound,1);
    end
    % 计算种群所有个体的适应函数值（fitnessfun的包装）
    function [pop_fitness, pop_fitvec] = obj_evalFitness(pop)
        if nargin == 0
            pop=obj_pop;
        end
        pop_fitness=zeros(size(pop,1),1);
        pop_fitvec=[];
        for i=1:size(pop,1)
            solution=restore_bound(pop(i,:));
            %assert(solution(end)>0)
            [pop_fitness(i), pop_fitvec(i,:)]= fitnessfun(solution);
        end
        %pop_fitness = [ones(length(pop),1) pop pop.^2] * [1 1 -1]';
    end
    % 生成初始天牛群和天牛步
    function [obj_pop, obj_pop_step, obj_pop_v] = initialPop()
        %obj_solution_bound = [-10, 10];
        obj_pop = rand(obj_sizepop,obj_dimension); 
        obj_pop_step = zeros(obj_sizepop,obj_dimension); 
        obj_pop_v = obj_Vmax*rands(obj_sizepop,obj_dimension); 
    end
    % 生成初始个体极值和群体极值
    function [obj_pop_fitness, obj_pop_fitvec, obj_zbest, obj_gbest, obj_gbest_fitness, obj_zbest_fitness] = initialFitness()
        obj_pop_fitness = zeros(obj_sizepop,1);
        obj_pop_fitvec=[];
        [bestfitness, bestindex] = max(obj_pop_fitness);
        obj_zbest = obj_pop(bestindex,:);
        obj_gbest = obj_pop;
        obj_gbest_fitness = obj_pop_fitness;
        obj_zbest_fitness = bestfitness;
    end
    function updatePop()
        for j = 1:obj_sizepop
            % 原始步(速度)更新
            obj_pop_v(j,:) = obj_pop_v(j,:) + obj_c1*rand*(obj_gbest(j,:) - obj_pop(j,:)) + obj_c2*rand*(obj_zbest - obj_pop(j,:));
            obj_pop_v(j,obj_pop_v(j,:)>obj_Vmax) = obj_Vmax;
            obj_pop_v(j,obj_pop_v(j,:)<-obj_Vmax) = -obj_Vmax;
            % 增量步更新
            dir = obj_pop_v(j,:)/(eps+norm(obj_pop_v(j,:)));
            xleft=handle_border(obj_pop(j,:)+dir*(obj_ante_dis/2));
            fleft=obj_evalFitness(xleft);
            xright=handle_border(obj_pop(j,:)-dir*(obj_ante_dis/2));
            fright=obj_evalFitness(xright);
            obj_pop_step(j,:)=obj_step_length*dir*sign(fleft-fright);
            % 种群更新
            obj_pop(j,:) = handle_border(obj_pop(j,:) + obj_lamda*obj_pop_v(j,:) + (1-obj_lamda)*obj_pop_step(j,:));
        end
    end
    function updateFitness()
    % 最优适应度值更新
        [obj_pop_fitness, obj_pop_fitvec] = obj_evalFitness(obj_pop);
        for j = 1:obj_sizepop
            % 个体最优更新
            if obj_pop_fitness(j) > obj_gbest_fitness(j)
                obj_gbest(j,:) = obj_pop(j,:);
                obj_gbest_fitness(j) = obj_pop_fitness(j);
            end
            % 群体最优更新
            if obj_pop_fitness(j) > obj_zbest_fitness
                obj_zbest = obj_pop(j,:);
                obj_zbest_fitness = obj_pop_fitness(j);
            end
        end
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
        lower = obj_solution_bound(:,1)';
        upper = obj_solution_bound(:,2)';
        result = (upper-lower).*solution+lower;
        assert(isequal(size(result),[1,obj_dimension]))
    end
end

