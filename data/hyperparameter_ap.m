% 超参数汇总：
function result = hyperparameter_ap()
    result.ap1_delta=0.03; %控制相邻规划点上各关节相较上一个规划点的活动范围
    result.ap1_obflag=true; %是否开启避障(耗时)
    result.ap1_to1=1/3; %fdt(偏离目标值)代价与oa(避障部分)的混合比例，tradeoff
    result.ap1_to2=0; %前两者(fdt和oa)与fq(前一位姿与后一位姿距离)的混合比例，tradeoff
    
    result.ap2_tradeoff=[1 0 1 1 1 1]; %代价函数中各指标的混合比例
    result.ap2_maxv=pi; %关节速度限制
    result.ap2_maxa=pi/2; %关节加速度限制
    result.ap2_obflag=true; %是否开启避障(耗时)
    
    result.ob_e=1e-2; %避障部分的最小距离，低于此最小距离则代价不再增长
    result.ob_beta=1; %避障部分代价函数中的指数系数
end

