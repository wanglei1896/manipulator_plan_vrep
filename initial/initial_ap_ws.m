%% 在matlab本地初始化各along path规划数据
global  outputData ...
        inputData ...
        optimLog ...       %优化日志，算法优化过程中产生的信息（用于分析）
        %model           %机械臂的运动学模型

isTest=false;
fromVrep=false;
%%% 初始化
%model = model_ur5();
optimLog = optimLog_ap(2);   %优化有几个组
inputData = input_ap([0.77068126,0.77309632,0.75121748,0.70708108,0.64506137,0.57133377,0.49313977,0.41797850,0.35329452,0.30551067,0.27931854,0.27690336,0.29878202,0.34291825,0.40493801,0.47866556,0.55685973,0.63202107,0.69670522,0.74448907,0.77068126;
                      0.15366602,0.23186007,0.30702132,0.37170541,0.41948923,0.44568133,0.44809639,0.42621762,0.38208121,0.32006153,0.24633414,0.16814020,0.092979006,0.028294884,-0.019489162,-0.045681395,-0.048096560,-0.026217796,0.017918654,0.079938300,0.15366602;
                      0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057,0.076850057
                      ], optimLog.group_num*10); %输入路径规范化后的采样段数
% 元循环，将上次优化的结果路径作为本次的目标路径
if ~isequal(outputData,[]) && ~isequal(outputData.endPath,[]) && isTest
    inputData=input_ap(outputData.endPath,20);
end
outputData = output_multiSeg();

%%% 默认
outputData.spacenum = optimLog.group_num*10;

%%% 从vrep中读取数据
if fromVrep
    vrep=remApi('remoteApi')'; % using the prototype file (remoteApiProto.m)
    fromVrep_ap;
end

if size(inputData.path,1)==3
    inputData.pStart = eye(4);
    inputData.pStart(1:3,4) = inputData.path(:,1);
    %inputData.qStart = model.ikunc(inputData.pStart);
end
