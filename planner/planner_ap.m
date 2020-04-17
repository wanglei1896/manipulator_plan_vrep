%% along path 的规划过程
% 
global outputData inputData optimLog model fitnessFun
%q0=initialJoint+[-pi/2, -pi/2, 0, -pi/2, 0, -pi/2];%初始关节角

%% optimLog更新，因此重置受其影响的变量
% optimLog先清空，再在analyze/reprsent_ap.m中计算相应值
optimLog = optimLog_ap(optimLog.group_num);

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_ap(model);
fitnessFun.parameter_bound=[-pi, pi; -pi, pi; % q * 6
                            -pi, pi; -pi, pi;
                            -pi, pi; -pi, pi;
                            -pi/4, pi/4; -pi/4, pi/4; % vq * 6
                            -pi/4, pi/4; -pi/4, pi/4;
                            -pi/4, pi/4; -pi/4, pi/4;
                            0.1, 10]; % time
fitnessFun.spacenum = outputData.spacenum/optimLog.group_num;

% 为各组天牛的参数初始化值
% qTable的第一列为起始端点，后每一列为每段的右端点
fitnessFun.qTable = initial_parameters(inputData.qStart, inputData.path(:,end), model);
optimLog.qTable_history=fitnessFun.qTable;

%% 主规划过程
main();

function main()
global inputData outputData optimLog fitnessFun
    %% 算法初始化
    sizepop = 50;
    iternum = 50;
    groupnum = optimLog.group_num;

    %% 调用算法规划
    disp('planning start.');
    optimLog.round_num=2;
    % 第一轮
    for ii=1:optimLog.round_num
    for i=1:2:groupnum
        fitnessFun.serial_number = i;
        path_index=equalDivide(inputData.spacenum,groupnum,i);
        fitnessFun.target_path = inputData.path(:,path_index);
        [fitness_history, fitvec_history,...
            solution_history,optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
        optimLog.group(i).fitness_history=[optimLog.group(i).fitness_history;fitness_history];
        optimLog.group(i).fitvec_history=[optimLog.group(i).fitvec_history;fitvec_history];
        optimLog.group(i).solution_history=[optimLog.group(i).solution_history;solution_history];
        last_solution = optimLog.group(i).solution_history(end,:);
        [~, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        % 更新qTable每段右端点
        fitnessFun.qTable.q(:,i+1) = last_result(1:6,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,i+1) = last_result(7:12,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,i+1) = last_result(13:18,fitnessFun.spacenum+1);
    end
    % 第二轮
    for i=2:2:groupnum
        fitnessFun.serial_number = i;
        path_index=equalDivide(inputData.spacenum,groupnum,i);
        fitnessFun.target_path = inputData.path(:,path_index);
        [fitness_history, fitvec_history,...
             solution_history, optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
        optimLog.group(i).fitness_history=[optimLog.group(i).fitness_history;fitness_history];
        optimLog.group(i).fitvec_history=[optimLog.group(i).fitvec_history;fitvec_history];
        optimLog.group(i).solution_history=[optimLog.group(i).solution_history;solution_history];
        last_solution = optimLog.group(i).solution_history(end,:);
        [~, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        fitnessFun.qTable.q(:,i+1) = last_result(1:6,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,i+1) = last_result(7:12,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,i+1) = last_result(13:18,fitnessFun.spacenum+1);
    end
    end
    disp('planning ended');
    
    % 综合各段，得出最终轨迹(合并第一段和各偶数点优化的段)
    outputData.trajectory = fitnessFun.qTable.q(:,1);
    assert(size(outputData.trajectory,2)==1) %trajectory中角度应存在每列上
    outputData.segment_curtimes(1) = 0;
    for i=1:groupnum
        fitnessFun.serial_number = i;
        last_solution = optimLog.group(i).solution_history(end,:);
        [~, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        if i==1
            outputData.trajectory = [outputData.trajectory, last_result(1:6,2:fitnessFun.spacenum+1)];%舍弃各组的第一个点，以免重复
        elseif mod(i,2)==0
            outputData.trajectory = [outputData.trajectory, last_result(1:6,2:end)]; 
            % 轨迹中各段连接点处位置应与qTable中相同
            assert(isequal(outputData.trajectory(:,i*fitnessFun.spacenum+1)...
                ,fitnessFun.qTable.q(:,i+1)))
        end
        outputData.segment_times(i) = last_solution(end);
        outputData.segment_curtimes(i+1) = outputData.segment_curtimes(i)+outputData.segment_times(i);
    end
end

function qTable = initial_parameters(qStart, positionFinal, model)
global optimLog
    assert(isequal(size(positionFinal),[3,1]));
    assert(isequal(size(qStart),[1,6]));
    pFinal = [1 0 0 0;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];
    pFinal(1:3,4) = positionFinal;
    qFinal = model.ikunc(pFinal);
    [qStart,qFinal] = regular_JointPos(qStart,qFinal);
    [q, vq, aq] = jtraj(qStart,qFinal,optimLog.group_num+1);
    qTable.q = q'; qTable.vq = vq'; qTable.aq = aq';
    assert(size(qTable.q,1)==6);
end