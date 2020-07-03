% 自动生成figure

%% 配置任务
initial_ap;
%% 运行
planner_ap;
executeInVrep;
%% 保存原始数据
disp('计算analyze数据...')
calculateHistory();
disp("保存数据至figure/unformatted_data目录下...")
dataPath=[ProjectPath,'./figure/unformatted_data'];
save([dataPath,'/optimLog','1'],'optimLog')
save([dataPath,'/model','1'],'model')
save([dataPath,'/outputData','1'],'outputData')
%% 格式化生成各图像
load([dataPath,'/optimLog','1','.mat'])
load([dataPath,'/model','1','.mat'])
load([dataPath,'/outputData','1','.mat'])
cost_variation;
joint_variation;
end_variation;
oa_dt_variation;
matlab_plan;

vrep_plan;
mindis_variation;