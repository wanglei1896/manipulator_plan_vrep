%% along path 的规划过程
% 

function outputData=planner_ap2(model, jointPath, obstacles, hyperparameter)
global optimLog
    input_spacenum = size(jointPath,2)-1;
    group_size = optimLog.group_num;
%% 配置代价函数
    % 轨迹编码
    fitnessFun = fitnessFun_ap2(model);
    fitnessFun.hyperparameter=hyperparameter;
    for i=1:optimLog.group_num   % 为每组维护一个边界
        fitnessFun.parameter_bound(:,:,i)=[
                        ones(model.joint_num,1)*[-pi, pi]; % vq
                        ones(model.joint_num,1)*[-pi, pi]; % aq
                        0.1, 10]; % time
    end
    fitnessFun.spacenum = input_spacenum/optimLog.group_num;
    fitnessFun.obstacles=obstacles;

    % 为各组天牛的参数初始化值
    % qTable的第一列为起始端点，后每一列为每段的右端点
    fitnessFun.qTable = initial_parameters();
    optimLog.qTable_history=fitnessFun.qTable;
    
    %% 算法初始化
    sizepop = 40;
    iternum = 25;

    %% 调用算法规划
    disp('planning start.');
    % 第一轮,奇数编号结点
    for i=1:2:group_size
        update_solution(i)
    end
    % 第二轮,偶数编号结点
    for i=2:2:group_size
        update_solution(i)
    end
    disp('planning ended');
    
    get_trajectory();
    
    function update_solution(group_number)
        bound=fitnessFun.parameter_bound(:,:,group_number);
        bound=reshape(bound,size(bound,1),size(bound,2));

        fitnessFun.serial_number = group_number;
        path_index=equalDivide(input_spacenum,group_size,group_number);
        fitnessFun.target_path = jointPath(:,path_index);
        % 调用优化算法
        [fitness_history, fitvec_history,...
            solution_history,all_solution_history, optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, bound, @fitnessFun.fitnessf);
        
        optimLog.group(group_number).fitness_history=[optimLog.group(group_number).fitness_history;fitness_history];
        optimLog.group(group_number).fitvec_history=[optimLog.group(group_number).fitvec_history;fitvec_history];
        optimLog.group(group_number).solution_history=[optimLog.group(group_number).solution_history;solution_history];
        optimLog.group(group_number).all_solution_history=[optimLog.group(group_number).all_solution_history;all_solution_history];
        last_solution = optimLog.group(group_number).solution_history(end,:);
        last_result = fitnessFun.convertSolutionToTrajectory(last_solution);
        % 更新qTable每段右端点
        %fitnessFun.qTable.q(:,group_number+1) = last_result(1:model.joint_num,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,group_number+1) = last_result(model.joint_num+1:model.joint_num*2,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,group_number+1) = last_result(model.joint_num*2+1:model.joint_num*3,fitnessFun.spacenum+1);
    end

    function get_trajectory()
       % 综合各段，得出最终轨迹(合并第一段和各偶数点优化的段)
        outputData.trajectory = fitnessFun.qTable.q(:,1);
        assert(size(outputData.trajectory,2)==1) %trajectory中角度应存在每列上
        outputData.segment_times=zeros(1,group_size);
        outputData.segment_curtimes=zeros(1,group_size+1);
        outputData.segment_curtimes(1) = 0;
        for iter=1:group_size
            fitnessFun.serial_number = iter;
            last_solution = optimLog.group(iter).solution_history(end,:);
            last_result = fitnessFun.convertSolutionToTrajectory(last_solution);
            if iter==1
                outputData.trajectory = [outputData.trajectory, last_result(1:model.joint_num,2:fitnessFun.spacenum+1)];%舍弃各组的第一个点，以免重复
            elseif mod(iter,2)==0
                outputData.trajectory = [outputData.trajectory, last_result(1:model.joint_num,2:end)]; 
                % 轨迹中各段连接点处位置应与qTable中相同
                assert(norm(outputData.trajectory(:,iter*fitnessFun.spacenum+1)...
                    -fitnessFun.qTable.q(:,iter+1))<1e-4)
            end
            outputData.segment_times(iter) = last_solution(end);
            outputData.segment_curtimes(iter+1) = outputData.segment_curtimes(iter)+outputData.segment_times(iter);
        end
        outputData.spacenum=size(outputData.trajectory,2)-1;
        outputData.endPath=[];
        outputData.jointPath=jointPath;
    end
    function qTable = initial_parameters()
        assert(size(jointPath,1)==model.joint_num);
        for ii=1:optimLog.group_num+1
            path_index=equalDivide(input_spacenum,group_size,ii);
            qTable.q(:,ii)=jointPath(:,path_index(1));
        end
        qTable.vq=zeros(model.joint_num,optimLog.group_num+1);
        qTable.aq=zeros(model.joint_num,optimLog.group_num+1);
        assert(size(qTable.q,1)==model.joint_num);
    end
end

