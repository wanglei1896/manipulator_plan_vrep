%用于自动生成所有figure,参数化不同任务

function formatData(taskName, ProjectPath, figure_format)
    %% 格式化生成各图像
    dataPath=[ProjectPath,'/figure/unformatted_data'];
    load([dataPath,'/',taskName,'_optimLog.mat'])
    load([dataPath,'/',taskName,'_model.mat'])
    load([dataPath,'/',taskName,'_outputData.mat'])
    load([dataPath,'/',taskName,'_fromVrepData.mat'])
    load([dataPath,'/',taskName,'_inputData.mat'])
    current_figure=figure;
    cost_variation;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_cost_variation.',figure_format], figure_format);
    clf,
    joint_variation('q');
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_joint_variation_q.',figure_format], figure_format);
    joint_variation('v');
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_joint_variation_v.',figure_format], figure_format);
    joint_variation('a');
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_joint_variation_a.',figure_format], figure_format);
    clf,
    end_variation;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_end_variation.',figure_format], figure_format);
    clf,
    oa_dt_variation;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_oa_dt_variation.',figure_format], figure_format);
    clf,
    mindis_variation;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_mindis_variation.',figure_format], figure_format);
    clf,
    %close(current_figure);

    matlab_plan;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_matlab_plan.',figure_format], figure_format);
    clf,
    vrep_plan;
    saveas(current_figure, [ProjectPath,'./figure/result/',figure_format,'/',taskName,'_vrep_plan.',figure_format], figure_format);
    close(current_figure);
end

