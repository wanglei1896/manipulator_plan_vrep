%% 规划过程
%

format long;
global inputData model fitnessFun
%q0=initialJoint+[-pi/2, -pi/2, 0, -pi/2, 0, -pi/2];%初始关节角

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_p2p(model.km);
fitnessFun.parameter_bound=[-pi, pi; -pi, pi; % qm * 6
                            -pi, pi; -pi, pi;
                            -pi, pi; -pi, pi;
                            -pi/4, pi/4; -pi/4, pi/4; % vqm * 6
                            -pi/4, pi/4; -pi/4, pi/4;
                            -pi/4, pi/4; -pi/4, pi/4;
                            0.1, 10; 0.1, 10]; % t1, t2
fitnessFun.spacenum = 10;
fitnessFun.qStart = inputData.qStart; fitnessFun.qFinal =inputData.qFinal;
fitnessFun.obstacles = inputData.obstacles;
fitnessFun.linkShapes = model.shape;
fitnessFun.total_vn = model.num_shapeVertex;

%% 主规划过程
main();

%% optimLog更新，因此重置受其影响的变量
if exist('histo_t1_t2','var')
    clear histo_t1_t2
end

function main()
global outputData optimLog fitnessFun
    % 算法初始化
    sizepop = 10;
    iternum = 10;

    disp('planning start.');

    [optimLog.fitness_history, optimLog.fitvec_history, optimLog.solution_history,...
        optimLog.all_solution_history, optimization_time] ...
        = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
    last_solution = optimLog.solution_history(end,:);
    outputData.segment_times = last_solution(end-1:end);
    outputData.segment_curtimes = [0, last_solution(end-1), last_solution(end)+last_solution(end-1)];
    [last_status, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution,outputData.spacenum);
    outputData.trajectory = last_result(1:6,:);

    disp('planning ended');
end