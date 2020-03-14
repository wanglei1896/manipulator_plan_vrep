%������matlab�����ڲ�ģ���е�۵����У�����չʾ
%��Ҫʹ��matlab robotic toolboxʵ��
% 
global model outputData


%ʹ��toolbox�Դ��ĵ㵽��켣
outputData_bench.trajactory = model.jtraj(inputData.pStart,inputData.pFinal,20,'ikine',@model.ikunc)';


%��е������
model.plot(outputData.trajectory','trail',{'r'});

figure,
plotJoint_Time(outputData);

%�ؽ��˶�ͼ
function plotJoint_Time(data)
    segnum = length(data.segment_times);
    spacePerSeg=floor(data.spacenum/segnum);
    t = linspace(data.segment_curtimes(1),data.segment_curtimes(2),spacePerSeg+1);
    for i=2:segnum
        temp = linspace(data.segment_curtimes(i),data.segment_curtimes(i+1),spacePerSeg+1);
        t=[t, temp(2:end)];
    end
    plot(t, data.trajectory(1:6,:)','.');
    %plot(linspace(0,data.total_time,data.spacenum+1),data.trajectory(1:6,:)','.');
    legend joint1 joint2 joint3 joint4 joint5 joint6
    hold on,
    grid on,
    xticks(data.segment_curtimes) %��ʾ��Ҫָʾ��
    yticks(-2*pi:pi/2:2*pi) %ͬ��
    yticklabels({'-2\pi','-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
end