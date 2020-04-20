% 探索path tracking规划中路径长度对规划结果的影响
global inputData optimLog
initial_ap;
for i=1:5
    inputData.path=regular_path([[0.3;0.4;0], [0.3;0.4;0]+i/10*[0.3;-0.4;0]], inputData.spacenum); 
    for j=1:10
        planner_ap;
        represent_ap;
        for k=1:optimLog.round_num
            fitness_sum_log(i,k,j)=optimLog.sum.fitness_history(length(optimLog.sum.fitness_history)/optimLog.round_num*k);
        end
        disp(' ')
        disp(['progress: ',num2str(((i-1)*10+j)/50*100),'%'])
        disp(' ')
    end
end