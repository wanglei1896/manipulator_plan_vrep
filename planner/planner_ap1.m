%% path tracking规划的第一步
%   确定各个轨迹连接点处的关节位置
global inputData optimLog model fitnessFun

%% optimLog更新，因此重置受其影响的变量
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_ap1(model.km);
fitnessFun.joint_num=model.joint_num;
fitnessFun.jointPath=zeros(fitnessFun.joint_num,inputData.spacenum+1);
fitnessFun.parameter_bound=ones(fitnessFun.joint_num,1)*[-0.2*pi, 0.2*pi];  %q * 6
fitnessFun.obstacles=inputData.obstacles;
fitnessFun.linkShapes = model.shape;
for i=1:inputData.obstacle_num
    fitnessFun.obsCentre=[fitnessFun.obsCentre, inputData.obstacles(i).centre];
end
for i=1:length(model.shape)
    fitnessFun.linkCentre=[fitnessFun.linkCentre, model.shape(i).centre]; 
end

%% 主规划过程
main();

function main()
global optimLog fitnessFun model outputData inputData
    %% 算法初始化
    iternum = 100;
    assert(length(model.shape)==model.joint_num)
    p_spacenum=inputData.spacenum/optimLog.group_num;
    fitnessFun.jointPath(:,1)=inputData.qStart';
    outputData.junctionPos=zeros(model.joint_num, optimLog.group_num+1);
    outputData.junctionPos(:,1)=inputData.qStart';

    %% 调用算法规划
    disp('planning start.');
    for i=2:inputData.spacenum+1
        update_solution(i);
    end
    disp('planning ended');
    
    outputData.jointPath=fitnessFun.jointPath;
    fh_sum=optimLog.group(1).fitness_history;
    for i=2:optimLog.group_num
        fh_sum=fh_sum+optimLog.group(i).fitness_history;
    end
    optimLog.sum.fitness_history=fh_sum;
    
    plot_posture();
    
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
            outputData.junctionPos(:,ceil(iter/p_spacenum))=posture';
        end
        fitnessFun.jointPath(:,iter)=posture';
    end

    function plot_posture()
        % 展示规划结果
        model.km.plot(inputData.qStart,'trail',{'r'})
        hold on
        plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
        plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
        plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
        model.km.plot(outputData.jointPath','trail',{'r'},'delay',0.1)
    end
end
