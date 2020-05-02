%% along path 的规划过程
% 
global outputData inputData optimLog model fitnessFun joint_num

%% optimLog更新，因此重置受其影响的变量
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(6); %优化有几个组

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_ap(model.km);
for i=1:optimLog.group_num   % 为每组维护一个边界
    fitnessFun.parameter_bound(:,:,i)=[
                    -pi, pi; -pi, pi; % q * 6
                    -pi, pi; -pi, pi;
                    -pi, pi; -pi, pi;
                    -pi, pi; -pi, pi; % vq * 6
                    -pi, pi; -pi, pi;
                    -pi, pi; -pi, pi;
                    0.1, 10]; % time
end
fitnessFun.spacenum = 10;
fitnessFun.joint_num = joint_num;
fitnessFun.obstacles=inputData.obstacles;
fitnessFun.linkShapes = model.shape;
for i=1:inputData.obstacle_num
    fitnessFun.obsCentre=[fitnessFun.obsCentre, inputData.obstacles(i).centre];
end
for i=1:length(model.shape)
    fitnessFun.linkCentre=[fitnessFun.linkCentre, model.shape(i).centre]; 
end

% 为各组天牛的参数初始化值
% qTable的第一列为起始端点，后每一列为每段的右端点
fitnessFun.qTable = initial_parameters(inputData.qStart);
optimLog.qTable_history=fitnessFun.qTable;

%% 主规划过程
main();

function main()
global outputData optimLog fitnessFun joint_num
    %% 算法初始化
    sizepop = 100;
    iternum = 40;
    group_size = optimLog.group_num;

    %% 调用算法规划
    disp('planning start.');
    optimLog.round_num=1;
    
    for ii=1:optimLog.round_num
        %fitnessFun.qTable = initial_parameters(inputData.qStart, inputData.path(:,end), model);
        % 第一轮,奇数编号结点
        for i=1:2:group_size
            update_solution(i,ii)
        end
        % 第二轮,偶数编号结点
        for i=2:2:group_size
            update_solution(i,ii)
        end
    end
    disp('planning ended');
    
    get_trajectory();
    
    function update_solution(group_number,round)
        global inputData
        bound=fitnessFun.parameter_bound(:,:,group_number);
        bound=reshape(bound,size(bound,1),size(bound,2));
        if round>1
            % 利用上一轮的结果作为经验，简化本轮的优化
            last_round_sol = optimLog.group(group_number).solution_history(end,:);
            last_round_fitvec = optimLog.group(group_number).fitvec_history(end,:);
            assert(length(last_round_fitvec)==26 || length(last_round_fitvec)==16)
            %range=last_round_fitvec(16)*6;
            range=(bound(1,2)-bound(1,1))/4;
            disp([num2str(group_number),'  ',num2str(range)])
            bound(1:joint_num,1)=last_round_sol(1:joint_num)'-range;
            bound(1:joint_num,2)=last_round_sol(1:joint_num)'+range;
            fitnessFun.parameter_bound(:,:,group_number)=bound;
        end        

        fitnessFun.serial_number = group_number;
        path_index=equalDivide(inputData.spacenum,group_size,group_number);
        fitnessFun.target_path = outputData.junctionPos(:,path_index);
        % 调用优化算法
        [fitness_history, fitvec_history,...
            solution_history,all_solution_history, optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, bound, @fitnessFun.fitnessf);
        
        optimLog.group(group_number).fitness_history=[optimLog.group(group_number).fitness_history;fitness_history];
        optimLog.group(group_number).fitvec_history=[optimLog.group(group_number).fitvec_history;fitvec_history];
        optimLog.group(group_number).solution_history=[optimLog.group(group_number).solution_history;solution_history];
        optimLog.group(group_number).all_solution_history=[optimLog.group(group_number).all_solution_history;all_solution_history];
        last_solution = optimLog.group(group_number).solution_history(end,:);
        [~, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        % 更新qTable每段右端点
        fitnessFun.qTable.q(:,group_number+1) = last_result(1:joint_num,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,group_number+1) = last_result(joint_num+1:joint_num*2,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,group_number+1) = last_result(joint_num*2+1:joint_num*3,fitnessFun.spacenum+1);
    end

    function get_trajectory()
       % 综合各段，得出最终轨迹(合并第一段和各偶数点优化的段)
        outputData.trajectory = fitnessFun.qTable.q(:,1);
        assert(size(outputData.trajectory,2)==1) %trajectory中角度应存在每列上
        outputData.segment_curtimes(1) = 0;
        for iter=1:group_size
            fitnessFun.serial_number = iter;
            last_solution = optimLog.group(iter).solution_history(end,:);
            [~, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
            if iter==1
                outputData.trajectory = [outputData.trajectory, last_result(1:joint_num,2:fitnessFun.spacenum+1)];%舍弃各组的第一个点，以免重复
            elseif mod(iter,2)==0
                outputData.trajectory = [outputData.trajectory, last_result(1:joint_num,2:end)]; 
                % 轨迹中各段连接点处位置应与qTable中相同
                assert(norm(outputData.trajectory(:,iter*fitnessFun.spacenum+1)...
                    -fitnessFun.qTable.q(:,iter+1))<1e-4)
            end
            outputData.segment_times(iter) = last_solution(end);
            outputData.segment_curtimes(iter+1) = outputData.segment_curtimes(iter)+outputData.segment_times(iter);
        end 
    end
end

function qTable = initial_parameters(qStart)
global optimLog outputData inputData joint_num
    assert(isequal(size(qStart),[1,joint_num]));
    assert(size(outputData.junctionPos,2)==inputData.spacenum+1);
    qTable.q=qStart';
    for i=1:optimLog.group_num
        index=equalDivide(inputData.spacenum,optimLog.group_num,i+1);
        qTable.q=[qTable.q, outputData.junctionPos(1:joint_num,index(1))];
    end
    qTable.vq=zeros(joint_num,optimLog.group_num+1);
    qTable.aq=zeros(joint_num,optimLog.group_num+1);
    assert(size(qTable.q,1)==joint_num);
end