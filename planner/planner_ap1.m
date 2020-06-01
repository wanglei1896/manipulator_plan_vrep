%% path tracking规划的第一步
%   确定各个轨迹连接点处的关节位置

%%将规划算法封装成函数
function jointPath=planner_ap1(model, inputData, hyperparameter)
global optimLog
    %% 配置代价函数
    fitnessFun = fitnessFun_ap1(model);
    fitnessFun.hyperparameter=hyperparameter;
    fitnessFun.jointPath=zeros(model.joint_num,inputData.spacenum+1);
    fitnessFun.parameter_bound=ones(model.joint_num,1)*[-pi, pi]*hyperparameter.ap1_delta;  %delta_q
    fitnessFun.obstacles=inputData.obstacles;
    %% 算法初始化
    iternum = 300;
    assert(length(model.shape)==model.joint_num)
    p_spacenum=inputData.spacenum/optimLog.group_num;
    fitnessFun.jointPath(:,1)=inputData.qStart';

    %% 调用算法规划
    disp('planning start.');
    for i=2:inputData.spacenum+1
        update_solution(i);
        optimLog.progress=(i-1)/inputData.spacenum*0.5;
    end
    disp('planning ended');
    
    jointPath=fitnessFun.jointPath;
    fh_sum=optimLog.group(1).fitness_history;
    for i=2:optimLog.group_num
        fh_sum=fh_sum+optimLog.group(i).fitness_history;
    end
    optimLog.sum.fitness_history=fh_sum;
    
    %plot_posture();
    
    function update_solution(iter)
        fitnessFun.previousJPos = fitnessFun.jointPath(:,iter-1);
        fitnessFun.target_pos = inputData.path(:,iter);

        % 调用优化算法
        [fitness_history, fitvec_history,solution_history] ...
            = AlgorithmBAS_fun(iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
        posture=fitnessFun.previousJPos+solution_history(end,:)';
        if mod(iter,p_spacenum)==1
            optimLog.group(floor(iter/p_spacenum)).fitness_history=fitness_history;
            optimLog.group(floor(iter/p_spacenum)).fitvec_history=fitvec_history;
            optimLog.group(floor(iter/p_spacenum)).solution_history=solution_history;
        end
        fitnessFun.jointPath(:,iter)=posture';
    end

    function plot_posture()
        % 展示规划结果
        plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
        hold on
        plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
        plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
        axis equal
        plotManipulator(model, outputData.trajectory(:,:),gca)
    end
end
