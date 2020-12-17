% 自动生成figure

%% 配置任务
initial_ap;
hyperparameter.ap1_obflag=true;
hyperparameter.ap2_obflag=false;
lambda11=[10,20];
for i=1:2
    disp(['task: ',inputData.task_name])
    disp(['parameter: \Lambda_{11}',num2str(lambda11(i),'%.1f')])
    hyperparameter.ap1_to1=lambda11(i)*1e-4;
    
%% 运行
    planner_ap;
    executeInVrep;
%% 保存原始数据
    saveData([inputData.task_name,'_',num2str(i)],ProjectPath);
%% 格式化生成各图像
    formatData([inputData.task_name,'_',num2str(i)],ProjectPath);
end