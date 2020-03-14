%ALGORITHMBSO_FUN �˴���ʾ�йش˺�����ժҪ
%       AlgorithmBSO�ĺ�����װ�汾
%       ����ص�����fitnessfun��handle����������

% ���ص�fitness_history��solution_history����ÿ�ε����е���Ⱥ���Ÿ�������
function [fitness_history, solution_history, optimization_time] = AlgorithmBSO_fun(sizepop,iteration_num,solution_bound,fitnessfun)
    %% properties
    obj_sizepop=50;
    obj_iteration_num=100;
    obj_precision=0.001;
    
    obj_decay_rate=0.98; %˥����
    obj_ante_dis=3; %antenna distance
    obj_step_lenth=1; %��ţ�Ĳ���
                     %ѧϰ����
    obj_c1=1.49445;
    obj_c2=1.49445;
    %�������ӣ�����PSO�е��ٶ���BAS�в����Ļ�ϱ���
    obj_lamda=0.2;
    %��ţ�ٶ�����
    obj_Vmax = 1;
    obj_Vmin = -1;
    
%     obj_pop;
%     obj_pop_step;  %��ţȺԭʼ��
%     obj_pop_v; %��ţȺ������
%     obj_pop_fitness;
%     obj_gbest; %��Ⱥ���弫ֵ��
%     obj_gbest_fitness;
%     obj_zbest; %��Ⱥȫ�弫ֵ��
%     obj_zbest_fitness;
    obj_sizepop = sizepop;
    obj_iteration_num = iteration_num;
    obj_solution_bound = solution_bound;
    [obj_pop, obj_pop_step, obj_pop_v] = initialPop();
    [obj_pop_fitness, obj_zbest, obj_gbest, obj_gbest_fitness, obj_zbest_fitness] = initialFitness();
    
    fitness_history = zeros(obj_iteration_num, 1);
    solution_history = zeros(obj_iteration_num, obj_dimension);
    
    %% main procedure
    tic
    % ����Ѱ��
    for ii=1:obj_iteration_num
        updatePop();
        updateFitness();
        %����˥��
        obj_ante_dis = obj_ante_dis * obj_decay_rate + obj_precision;
        obj_step_lenth = obj_step_lenth * obj_decay_rate;
        % �洢��ǰ�������Ž��
        [best_fit,best_index]=max(obj_pop_fitness);
        fitness_history(ii)=1/best_fit;
        bsol=obj_pop(best_index,:);
        solution_history(ii,:)=bsol;
    end
    toc
    optimization_time=toc;
    
    %% subfunction
    function dimension = obj_dimension()
        dimension = size(obj_solution_bound,1);
    end
    % ������Ⱥ���и������Ӧ����ֵ��fitnessfun�İ�װ��
    function pop_fitness = obj_evalFitness(pop)
        if nargin == 0
            pop=obj_pop;
        end
        pop_fitness=zeros(size(pop,1),1);
        for i=1:size(pop,1)
            assert(pop(i,end-1)>0 && pop(i,end)>0);
            pop_fitness(i) = fitnessfun(pop(i,:));
        end
        %pop_fitness = [ones(length(pop),1) pop pop.^2] * [1 1 -1]';
    end
    % ���ɳ�ʼ��ţȺ����ţ��
    function [obj_pop, obj_pop_step, obj_pop_v] = initialPop()
        %obj_solution_bound = [-10, 10];
        obj_pop = (ones(obj_sizepop,1)*(obj_solution_bound(:,2)-obj_solution_bound(:,1))')...
                  .*(rand(obj_sizepop,obj_dimension))+(ones(obj_sizepop,1)*obj_solution_bound(:,1)'); 
        obj_pop_step = rands(obj_sizepop,obj_dimension); 
        obj_pop_v = rands(obj_sizepop,obj_dimension); 
    end
    % ���ɳ�ʼ���弫ֵ��Ⱥ�弫ֵ
    function [obj_pop_fitness, obj_zbest, obj_gbest, obj_gbest_fitness, obj_zbest_fitness] = initialFitness()
        obj_pop_fitness = zeros(obj_sizepop,1);
        [bestfitness, bestindex] = max(obj_pop_fitness);
        obj_zbest = obj_pop(bestindex,:);  
        obj_gbest = obj_pop;
        obj_gbest_fitness = obj_pop_fitness; 
        obj_zbest_fitness = bestfitness;
    end
    function updatePop()
        for j = 1:obj_sizepop
            % �ٶȸ���
            obj_pop_v(j,:) = obj_pop_v(j,:) + obj_c1*rand*(obj_gbest(j,:) - obj_pop(j,:)) + obj_c2*rand*(obj_zbest - obj_pop(j,:));
            obj_pop_v(j,obj_pop_v(j,:)>obj_Vmax) = obj_Vmax;
            obj_pop_v(j,obj_pop_v(j,:)<obj_Vmin) = obj_Vmin;
            % ��������
            dir = obj_pop_v(j,:)/(eps+norm(obj_pop_v(j,:)));
            xleft=handle_border(obj_pop(j,:)+dir*(obj_ante_dis/2));
            fleft=obj_evalFitness(xleft);
            xright=handle_border(obj_pop(j,:)-dir*(obj_ante_dis/2));
            fright=obj_evalFitness(xright);
            
            obj_pop_step(j,:)=obj_step_lenth*dir*sign(fleft-fright);
            % ��Ⱥ����
            obj_pop(j,:) = handle_border(obj_pop(j,:) + obj_lamda*obj_pop_v(j,:) + (1-obj_lamda)*obj_pop_step(j,:));
%             max_i=find(obj_pop(j,:)>obj_solution_bound(:,2)');
%             obj_pop(j,max_i) = obj_solution_bound(max_i,2)';
%             min_i=find(obj_pop(j,:)<obj_solution_bound(:,1)');
%             obj_pop(j,min_i) = obj_solution_bound(min_i,1)';
        end
    end
    function updateFitness()
    % ��Ӧ��ֵ����
        obj_pop_fitness = obj_evalFitness(obj_pop);
        for j = 1:obj_sizepop
            % �������Ÿ���
            if obj_pop_fitness(j) > obj_gbest_fitness(j)
                obj_gbest(j,:) = obj_pop(j,:);
                obj_gbest_fitness(j) = obj_pop_fitness(j);
            end
            % Ⱥ�����Ÿ���
            if obj_pop_fitness(j) > obj_zbest_fitness
                obj_zbest = obj_pop(j,:);
                obj_zbest_fitness = obj_pop_fitness(j);
            end
        end
    end
    %�߽紦��
    function outputvec = handle_border(inputvec)
        outputvec = inputvec;
        max_i=find(inputvec>obj_solution_bound(:,2)');
        outputvec(max_i) = obj_solution_bound(max_i,2)';
        min_i=find(inputvec<obj_solution_bound(:,1)');
        outputvec(min_i) = obj_solution_bound(min_i,1)';
    end
end

