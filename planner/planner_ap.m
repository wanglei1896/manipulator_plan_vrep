%% along path �Ĺ滮����
% 
global outputData inputData optimLog model fitnessFun
%q0=initialJoint+[-pi/2, -pi/2, 0, -pi/2, 0, -pi/2];%��ʼ�ؽڽ�

%% ���ô��ۺ���
% �켣����
fitnessFun = fitnessFun_ap(model);
fitnessFun.parameter_bound=[-pi, pi; -pi, pi; % q * 6
                            -pi, pi; -pi, pi;
                            -pi, pi; -pi, pi;
                            -pi/4, pi/4; -pi/4, pi/4; % vq * 6
                            -pi/4, pi/4; -pi/4, pi/4;
                            -pi/4, pi/4; -pi/4, pi/4;
                            0.1, 10]; % time
fitnessFun.spacenum = outputData.spacenum/optimLog.group_num;

% Ϊ������ţ�Ĳ�����ʼ��ֵ
% qTable�ĵ�һ��Ϊ��ʼ�˵㣬��ÿһ��Ϊÿ�ε��Ҷ˵�
fitnessFun.qTable = initial_parameters(inputData.qStart, inputData.path(:,end), model);

%% ���滮����
main();

%% optimLog���£������������Ӱ��ı���
if exist('histo_t1_t2','var')
    clear histo_t1_t2
end

function main()
global inputData outputData optimLog fitnessFun
    %% �㷨��ʼ��
    sizepop = 10;
    iternum = 50;
    groupnum = optimLog.group_num;

    %% �����㷨�滮
    disp('planning start.');
    spacePerGroup = inputData.spacenum/groupnum;
    % ��һ��
    for i=1:2:groupnum
        fitnessFun.serial_number = i;
        fitnessFun.target_path = inputData.path(:,1:spacePerGroup+(i-1)*spacePerGroup);
        [optimLog.group(i).fitness_history, optimLog.group(i).solution_history, optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
        last_solution = optimLog.group(i).solution_history(end,:);
        [last_status, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        % ����qTableÿ���Ҷ˵�
        fitnessFun.qTable.q(:,i+1) = last_result(1:6,end);
        fitnessFun.qTable.vq(:,i+1) = last_result(7:12,end);
        fitnessFun.qTable.aq(:,i+1) = last_result(13:18,end);
    end
    % �ڶ���
    for i=2:2:groupnum
        fitnessFun.serial_number = i;
        fitnessFun.target_path = inputData.path(:,1:spacePerGroup+(i-1)*spacePerGroup);
        [optimLog.group(i).fitness_history, optimLog.group(i).solution_history, optimization_time] ...
            = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
    end
    disp('planning ended');
    
    % �ۺϸ��Σ��ó����չ켣
    outputData.trajectory = inputData.qStart';
    assert(size(outputData.trajectory,2)==1) %trajectory�нǶ�Ӧ����ÿ����
    outputData.segment_curtimes(1) = 0;
    for i=1:groupnum
        last_solution = optimLog.group(i).solution_history(end,:);
        [last_status, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
        outputData.segment_times(i) = last_solution(13);
        outputData.segment_curtimes(i+1) = outputData.segment_curtimes(i)+outputData.segment_times(i);
        outputData.trajectory = [outputData.trajectory, last_result(1:6,2:end)]; %��������ĵ�һ���㣬�����ظ�
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
    [q, vq, aq] = jtraj(qStart,qFinal,optimLog.group_num+1);
    qTable.q = q'; qTable.vq = vq'; qTable.aq = aq';
    assert(size(qTable.q,1)==6);
end