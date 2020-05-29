% 超参数汇总：
function result = hyperparameter_ap()
    result.ap1_delta=0.1; %控制相邻规划点上各关节相较上一个规划点的活动范围
    result.ap1_to1=0; %fq与cost的混合比例，tradeoff
    result.ap1_obflag=true; %是否开启避障(耗时)
    result.ap1_to2=1/3; %避障部分与fdt代价的混合比例，tradeoff
    
    result.ap2_tradeoff=[1 0 1 1 1 1]; %代价函数中各指标的混合比例
    result.ap2_obflag=true; %是否开启避障(耗时)
    
    result.ob_e=1e-3; %避障部分的最小距离，低于此最小距离则代价不再增长
    result.ob_beta=1; %避障部分代价函数中的指数系数
end

