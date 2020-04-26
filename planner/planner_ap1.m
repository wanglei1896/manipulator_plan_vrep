%% path tracking规划的第一步
%   确定各个轨迹连接点处的关节位置
global inputData optimLog model fitnessFun

%% optimLog更新，因此重置受其影响的变量
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_ap1(model.km);
fitnessFun.parameter_bound=[
                    -pi, pi; -pi, pi; % q * 6
                    -pi, pi; -pi, pi;
                    -pi, pi; -pi, pi];
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
global optimLog fitnessFun model outputData
    %% 算法初始化
    iternum = 500;
    group_size = optimLog.group_num;
    assert(length(model.shape)==6)
    spare_num=10; %要生成的备选解个数
    outputData.junctionPos=zeros(group_size,6,spare_num);

    %% 调用算法规划
    disp('planning start.');
    for i=1:group_size
        update_solution(i);
    end
    disp('planning ended');
    
    plot_posture();
    
    function update_solution(group_number)
        global inputData
        fitnessFun.serial_number = group_number;
        path_index=equalDivide(inputData.spacenum,group_size,group_number+1);
        fitnessFun.target_pos = inputData.path(:,path_index(1));
        count=0;
        while count<spare_num
            % 调用优化算法
            [fitness_history, fitvec_history,solution_history,optimization_time] ...
                = AlgorithmBAS_fun(iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
            %optimLog.group(group_number).fitness_history=[optimLog.group(group_number).fitness_history;fitness_history];
            %optimLog.group(group_number).fitvec_history=[optimLog.group(group_number).fitvec_history;fitvec_history];
            %optimLog.group(group_number).solution_history=[optimLog.group(group_number).solution_history;solution_history];
            posture=solution_history(end,:)';
            fit=fitness_history(end);
            if fit<1e-3
                count=count+1;
                outputData.junctionPos(group_number,:,count)=posture;
            end
        end
    end

    function plot_posture()
        global inputData
        % 展示规划结果
        model.km.plot(inputData.qStart,'trail',{'r'})
        hold on
        plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
        plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
        plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
        model.km.plot(outputData.junctionPos(:,:,1),'trail',{'r'},'delay',1)
    end
end
