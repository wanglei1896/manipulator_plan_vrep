%用于自动生成所有figure,参数化不同任务

function formatData(taskName, ProjectPath)
    %% 格式化生成各图像
    dataPath=[ProjectPath,'/figure/unformatted_data'];
    load([dataPath,'/',taskName,'_optimLog.mat'])
    load([dataPath,'/',taskName,'_model.mat'])
    load([dataPath,'/',taskName,'_outputData.mat'])
    load([dataPath,'/',taskName,'_fromVrepData.mat'])
    load([dataPath,'/',taskName,'_inputData.mat'])
    current_figure=figure;
    cost_variation;
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_cost_variation.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_cost_variation.eps'], 'epsc');
    clf,
    joint_variation('q');
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_joint_variation_q.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_joint_variation_q.eps'], 'epsc');
    joint_variation('v');
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_joint_variation_v.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_joint_variation_v.eps'], 'epsc');
    joint_variation('a');
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_joint_variation_a.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_joint_variation_a.eps'], 'epsc');
    clf,
    end_variation;
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_end_variation.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_end_variation.eps'], 'epsc');
    clf,
    oa_dt_variation;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_oa_dt_variation.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_oa_dt_variation.eps'], 'epsc');
    clf,
    mindis_variation;
    current_figure.CurrentAxes.FontSize=16;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_mindis_variation.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_mindis_variation.eps'], 'epsc');
    clf,
    %close(current_figure);

    matlab_plan;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_matlab_plan.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_matlab_plan.eps'], 'epsc');
    clf,
    vrep_plan;
    saveas(current_figure, [ProjectPath,'./figure/result/fig','/',taskName,'_vrep_plan.fig'], 'fig');
    saveas(current_figure, [ProjectPath,'./figure/result/eps','/',taskName,'_vrep_plan.eps'], 'epsc');
    close(current_figure);
end

