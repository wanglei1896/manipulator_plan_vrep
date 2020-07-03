% 格式化避障指标和沿轨迹精度指标随迭代次数变化图
calcu_oa_dt();
plot([optimLog.sum.dt_history,optimLog.sum.oa_history]);

function calcu_oa_dt()
global optimLog
    iternum=length(optimLog.group(1).fitness_history);
    sum1=zeros(iternum,1);
    sum2=zeros(iternum,1);
    for i=1:2:optimLog.group_num
        sum1=sum1+optimLog.group(i).fitvec_history(:,3);
    end
    for i=2:2:optimLog.group_num
        sum2=sum2+optimLog.group(i).fitvec_history(:,3);
    end
    optimLog.sum.dt_history=[sum1;sum2];
    sum1=zeros(iternum,1);
    sum2=zeros(iternum,1);
    for i=1:2:optimLog.group_num
        sum1=sum1+optimLog.group(i).fitvec_history(:,4);
    end
    for i=2:2:optimLog.group_num
        sum2=sum2+optimLog.group(i).fitvec_history(:,4);
    end
    optimLog.sum.oa_history=[sum1;sum2];
end