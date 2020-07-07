

function saveData(taskName, ProjectPath)
global optimLog model outputData fromVrepData inputData hyperparameter
    %% 保存原始数据
    disp('calculate analyze data...')
    calculateHistory();
    disp("save data to figure/unformatted_data...")
    dataPath=[ProjectPath,'/figure/unformatted_data'];
    save([dataPath,'/',taskName,'_optimLog.mat'],'optimLog')
    save([dataPath,'/',taskName,'_model.mat'],'model')
    save([dataPath,'/',taskName,'_outputData.mat'],'outputData')
    save([dataPath,'/',taskName,'_fromVrepData.mat'],'fromVrepData')
    save([dataPath,'/',taskName,'_inputData.mat'],'inputData')
    save([dataPath,'/',taskName,'_hyperparameter.mat'],'hyperparameter')
end

