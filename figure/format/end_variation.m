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
    lgd=legend('x_x(t)', 'x_y(t)', 'x_z(t)');
    lgd.NumColumns=3;
    hold on,
    grid on,
    xticks(round(outputData.segment_curtimes,2)) %显示重要指示线
    ax=gca;
    ax.FontSize=12;
    ax.XLabel.String='Time (sec)';
    ax.YLabel.String='End-effector Position (m)';
    ax.XLim(2)=outputData.segment_curtimes(end)+0.01;
    ax.YLim=[-0.5,1.0];
end
