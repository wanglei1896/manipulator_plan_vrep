% 自动生成figure

%% 配置任务
initial_ap;
%hyperparameter.ap1_obflag=false;
hyperparameter.ap2_obflag=false;
for lambda11=[1,2]
    disp(['task: ',inputData.task_name])
    disp(['parameter: \Lambda_{11}',num2str(lambda11,'%.1f')])
    hyperparameter.ap2tradeoff(3)=lambda11;
%% 运行
    planner_ap;
    executeInVrep;
%% 保存原始数据
    saveData([inputData.task_name,'_',num2str(lambda11,'%.1f')],ProjectPath);
%% 格式化生成各图像
    formatData([inputData.task_name,'_',num2str(lambda11,'%.1f')],ProjectPath,'fig');
end