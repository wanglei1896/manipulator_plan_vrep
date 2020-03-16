%% 规划过程
%

format long;
global outputData inputData optimLog model fitnessFun
%q0=initialJoint+[-pi/2, -pi/2, 0, -pi/2, 0, -pi/2];%初始关节角

%% 配置代价函数
% 轨迹编码
fitnessFun = fitnessFun_p2p(model);
fitnessFun.parameter_bound=[-pi, pi; -pi, pi; % qm * 6
                            -pi, pi; -pi, pi;
                            -pi, pi; -pi, pi;
                            -pi/4, pi/4; -pi/4, pi/4; % vqm * 6
                            -pi/4, pi/4; -pi/4, pi/4;
                            -pi/4, pi/4; -pi/4, pi/4;
                            0.1, 10; 0.1, 10]; % t1, t2
fitnessFun.spacenum = outputData.spacenum;
fitnessFun.qStart = inputData.qStart; fitnessFun.qFinal =inputData.qFinal;

%% 算法初始化
%{ 
iter=length(t);
initx=pStart(1,4);
inity=pStart(2,4);
initz=pStart(3,4);
% 期望位置与实际位置
diseredPosition=zeros(3,iter);
actualPosition=zeros(3,iter);
% 输出的关节运动序列
outputjointValue=zeros(6,iter);
%}
sizepop = 10;
iternum = 100;


%% 调用算法规划
disp('planning start.');

[optimLog.fitness_history, optimLog.solution_history, optimization_time] ...
    = AlgorithmBSO_fun(sizepop, iternum, fitnessFun.parameter_bound, @fitnessFun.fitnessf);
last_solution = optimLog.solution_history(end,:);
outputData.segment_times = last_solution(end-1:end);
outputData.segment_curtimes = [0, last_solution(end-1), last_solution(end)+last_solution(end-1)];
[last_status, last_result] = fitnessFun.convertSolutionToTrajectory(last_solution);
outputData.trajectory = last_result(1:6,:);

%{
for k=1:iter
    pt=eye(4);
    diseredPosition(1,k)=0.3*sin(pi*t(k)/5);%0.1*sin(pi*t(k)/5)*cos(sin(3*pi*t(k)/10))+initx-0.01*sin(pi*0/5)*cos(sin(3*pi*0/10));
    diseredPosition(2,k)=0.3*cos(pi*t(k)/5);%0.1*sin(pi*t(k)/5)*sin(sin(3*pi*t(k)/10))+inity-0.01*sin(pi*0/5)*sin(sin(3*pi*0/10));
    diseredPosition(3,k)=initz+0.1*(sin(pi*t(k)));

    pt(1,4)=diseredPosition(1,k);
    pt(2,4)=diseredPosition(2,k);
    pt(3,4)=diseredPosition(3,k);

    try
        outputjointValue(:,k)=manipulator.ikunc(pt);
    catch e
        disp(e);
        disp(diseredPosition(:,k));
    end
    % drdx(k)=0.1*(pi/5*cos(pi*t(k)/5)*cos(sin(3*pi*t(k)/10))-3*pi/10*cos(3*pi*t(k)/10)*sin(sin(3*pi*t(k)/10))*sin(pi*t(k)/5));
    % drdy(k)=0.1*(pi/5*cos(pi*t(k)/5)*sin(sin(3*pi*t(k)/10))+3*pi/10*cos(3*pi*t(k)/10)*cos(sin(3*pi*t(k)/10))*sin(pi*t(k)/5));
    % drdz(k)=0;
    % [Ppx,Ppy,Ppz]=position(theta(:,k));
    % actualPosition(:,k)=[Ppx(6);Ppy(6);Ppz(6)];
end
%}
disp('planning ended');

% optimLog更新，因此重置受其影响的变量
if exist('histo_t1_t2','var')
    clear histo_t1_t2
end