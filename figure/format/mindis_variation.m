% 格式化机械臂距离障碍物的最小距离随时间变化图
plotMinDis();

function plotMinDis()
global fromVrepData
    plot(fromVrepData.mindis_variation)
    ax=gca;
    ax.FontSize=12;
    ax.YLabel.String = 'Minimum Distance (m)';
    ax.XLabel.String = 'Time (s)';
end