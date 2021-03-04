% 格式化机械臂距离障碍物的最小距离随时间变化图
plotMinDis();

function plotMinDis()
global fromVrepData outputData
    data=fromVrepData.mindis_variation(2:end);
    size=length(data);
    plot(linspace(0,outputData.segment_curtimes(end),size),data)
    hold on
    ax=gca;
    ax.XLim(2)=outputData.segment_curtimes(end);
    plot([0,ax.XLim(2)],[0.01,0.01],'--')
    ax.YLim(1) = 0;
    ax.YLabel.String = 'Minimum Distance (m)';
    ax.XLabel.String = 'Time (s)';
end