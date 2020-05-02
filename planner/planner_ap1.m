%% path tracking规划的第一步
%   确定各个轨迹连接点处的关节位置
global inputData optimLog model fitnessFun joint_num

joint_num = 6;
%% optimLog更新，因此重置受其影响的变量
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_ap1(model.km);
fitnessFun.spacenum=1;
fitnessFun.joint_num=joint_num;
fitnessFun.jointPath=zeros(fitnessFun.joint_num,inputData.spacenum+1);
fitnessFun.parameter_bound=ones(fitnessFun.joint_num*fitnessFun.spacenum,1)*[-2*pi, 2*pi];  %q * 6
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
global optimLog fitnessFun model outputData inputData joint_num
    %% 算法初始化
    iternum = 300;
    assert(length(model.shape)==joint_num)
    fitnessFun.jointPath(:,1)=inputData.qStart';

    %% 调用算法规划
    disp('planning start.');
    for i=1:inputData.spacenum
        update_solution(i);
    end
    disp('planning ended');
    
    outputData.junctionPos=fitnessFun.jointPath;
    fh_sum=optimLog.group(1).fitness_history;
    for i=2:optimLog.group_num
        fh_sum=fh_sum+optimLog.group(i).fitness_history;
    end
    optimLog.sum.fitness_history=fh_sum;
    
    plot_posture();
    
    function update_solution(group_number)
        fitnessFun.serial_number = group_number;
        fitnessFun.target_pos = inputData.path(:,group_number:group_number+1);

        % 调用优化算法
        [fitness_history, fitvec_history,solution_history,optimization_time] ...
            = AlgorithmBAS_fun(iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
        optimLog.group(group_number).fitness_history=[optimLog.group(group_number).fitness_history;fitness_history];
        optimLog.group(group_number).fitvec_history=[optimLog.group(group_number).fitvec_history;fitvec_history];
        optimLog.group(group_number).solution_history=[optimLog.group(group_number).solution_history;solution_history];
        posture=solution_history(end,:);

        fitnessFun.jointPath(:,group_number+1)=posture';
    end

    function plot_posture()
        % 展示规划结果
        model.km.plot(inputData.qStart,'trail',{'r'})
        hold on
        plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
        plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
        plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
        model.km.plot(outputData.junctionPos','trail',{'r'},'delay',1)
    end
end
