% 通过optimLog里各组的solutionhistory重新计算还原出每次迭代时的qTable
% ,以及path_history和sum.fitness_history
%% WATCH OUT! (潜在bug较多,小心使用)
function calculateHistory()
    global optimLog fitnessFun outputData inputData model
    n = optimLog.group_num;
    nj = size(optimLog.qTable_history(1).q, 1);
    round_num=1;
    ni = length(optimLog.group(1).fitness_history)/round_num;  %迭代次数
    fitnessFun.qTable = optimLog.qTable_history(1);
    tic
    for ii=1:round_num
        for i=1:ni
            if i==ni && ii==3
                disp('')
            end
            sum=0;
            for j=1:2:n
                [cost, firstSegCost]=handle_group(i,j);
                sum=sum+cost;
            end
            optimLog.qTable_history((ii-1)*ni*2+i+1)=fitnessFun.qTable;
            optimLog.sum.fitness_history((ii-1)*ni*2+i)=sum;
        end
        for i=(ni+1):(2*ni)
            optimLog.jointPath_history(:,:,1,(ii-1)*ni*2+i)=optimLog.jointPath_history(:,:,1,(ii-1)*ni*2+ni);
            optimLog.workPath_history(:,:,1,(ii-1)*ni*2+i)=optimLog.workPath_history(:,:,1,(ii-1)*ni*2+ni);
            sum=firstSegCost;  %第一段轨迹的代价值
            for j=2:2:n
                cost=handle_group(i,j);
                sum=sum+cost;
            end
            optimLog.qTable_history((ii-1)*ni*2+i+1)=fitnessFun.qTable;
            optimLog.sum.fitness_history((ii-1)*ni*2+i)=sum;
        end
    end
    toc
    endPath=optimLog.workPath_history(:,1,1,end);
    for i=1:n
        p=optimLog.workPath_history(:,:,i,end);
        endPath=[endPath,p(:,2:end)];
    end
    outputData.endPath=endPath;
    
    function [cost, firstSegCost]=handle_group(iter, group_num)
        fitnessFun.serial_number=group_num;
        path_index=equalDivide(inputData.spacenum,n,group_num);
        fitnessFun.target_path=outputData.jointPath(:,path_index);
        index=(ii-1)*ni+iter;
        if iter>ni
            index=index-ni;
        end
        solution=optimLog.group(group_num).solution_history(index,:);
        result=fitnessFun.convertSolutionToTrajectory(solution);
        fitnessFun.qTable.q(:,group_num+1)=result(1:nj,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,group_num+1)=result(nj+1:nj*2,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,group_num+1)=result(nj*2+1:nj*3,fitnessFun.spacenum+1);
        ql=result(1:model.joint_num,:);
        path=[];
        for q=ql
            Trans=fitnessFun.fastForwardTrans(q);
            path=[path,Trans(1:3,4,7)];
        end
        optimLog.jointPath_history(:,:,group_num,(ii-1)*ni*2+iter)=...
            regular_path(result(1:nj,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
        optimLog.workPath_history(:,:,group_num,(ii-1)*ni*2+iter)=...
            regular_path(path(:,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
        if group_num+1<=n
            optimLog.jointPath_history(:,:,group_num+1,(ii-1)*ni*2+iter)=...
            regular_path(result(1:nj,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
            optimLog.workPath_history(:,:,group_num+1,(ii-1)*ni*2+iter)=...
                regular_path(path(:,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
        end
        cost=1/fitnessFun.evaluateTrajectory(result,solution);
        if group_num==1 && ii==1
            assert(abs(cost-optimLog.group(1).fitness_history(iter))<1e-4,...
                "%d %f  %f\n",iter,cost,optimLog.group(1).fitness_history(iter));
        end
        if group_num==1 && iter==ni   %第一轮最后一次迭代时第一段轨迹的代价值
            fitnessFun.target_path=outputData.jointPath(:,path_index(1:ceil(length(path_index)/2)));
            %fitnessFun.target_path=fitnessFun.target_path(:,1:fitnessFun.spacenum+1);
            [fit, cost_vec]=fitnessFun.evaluateTrajectory(result(:,1:fitnessFun.spacenum+1),solution);
            firstSegCost=1/fit-cost_vec(end);
        else
            firstSegCost=-1;
        end
    end
end

