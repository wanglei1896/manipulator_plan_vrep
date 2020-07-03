% 格式化机械臂末端变化图
plotEndVariation()

function plotEndVariation()
global outputData
    segnum = length(outputData.segment_times);
    spacePerSeg=floor(outputData.spacenum/segnum);
    t = linspace(outputData.segment_curtimes(1),outputData.segment_curtimes(2),spacePerSeg+1);
    for i=2:segnum
        temp = linspace(outputData.segment_curtimes(i),outputData.segment_curtimes(i+1),spacePerSeg+1);
        t=[t, temp(2:end)];
    end
    plot(t, outputData.endPath','-');
    legend x_x(t) x_y(t) x_z(t)
    hold on,
    grid on,
    xticks(outputData.segment_curtimes) %显示重要指示线
    yticks(-2*pi:pi/2:2*pi) %同上
    %yticklabels({'-2\pi','-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
end
