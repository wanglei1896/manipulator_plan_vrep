% 格式化代价函数值随迭代次数变化图
%{
iternum=length(optimLog.group(1).fitness_history);
sum1=zeros(iternum,1);
sum2=zeros(iternum,1);
for i=1:2:optimLog.group_num
    sum1=sum1+optimLog.group(i).fitness_history;
end
for i=2:2:optimLog.group_num
    sum2=sum2+optimLog.group(i).fitness_history;
end
optimLog.sum.fitness_history=[sum1;sum2];
%}
plotCost();
function plotCost()
global optimLog
    plot(optimLog.sum.fitness_history)
    ax=gca;
    ax.FontSize=12;
    ax.YLabel.String = 'Cost Value';
    ax.XLabel.String = 'Iteration times';
end