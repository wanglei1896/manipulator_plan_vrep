%关节运动图,obj是为了适配appDesigner中的plot调用
function plotJoint_Time(data,obj)
    segnum = length(data.segment_times);
    spacePerSeg=floor(data.spacenum/segnum);
    t = linspace(data.segment_curtimes(1),data.segment_curtimes(2),spacePerSeg+1);
    for i=2:segnum
        temp = linspace(data.segment_curtimes(i),data.segment_curtimes(i+1),spacePerSeg+1);
        t=[t, temp(2:end)];
    end
    if nargin<=1
        plot(t, data.trajectory(1:6,:)','-');
        legend joint1 joint2 joint3 joint4 joint5 joint6
        hold on,
        grid on,
        xticks(data.segment_curtimes) %显示重要指示线
        yticks(-2*pi:pi/2:2*pi) %同上
        yticklabels({'-2\pi','-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
    else
        plot(obj,t,data.trajectory(1:6,:)','-');
        lgd=legend(obj,{'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'},'FontSize',10);
        lgd.NumColumns = 3;
        obj.XLabel.String = 'execution time (s)';
        obj.YLabel.String = 'joint angle (rad)';
        obj.XGrid = 'on';
        obj.XTick = roundn(data.segment_curtimes,-2);
        obj.YTick = -2*pi:pi/2:2*pi;
        obj.YTickLabel = {'-2\pi','-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'};
    end
end
