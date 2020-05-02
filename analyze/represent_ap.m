%% along path规划中优化过程的图形化展示
% 优化过程中各个解代表的机械臂末端路径的变化
%

%if isequal(optimLog.path_history,[])
    calculateHistoy();
%end

plotOthers();
%figure,
%plotInWS();
figure,
plotInCS('c');

function plotOthers()
global optimLog
	plot([optimLog.group(1).fitness_history, optimLog.group(2).fitness_history])
    legend group1 group2
    figure,
    plot([optimLog.group(1).fitvec_history(:,4),optimLog.group(2).fitvec_history(:,4)])
    legend group1 group2
    figure,
    plot(optimLog.group(1).fitvec_history(:,7:end));
    figure,
    plot(optimLog.group(2).fitvec_history(:,7:end));
    figure,
    plot(optimLog.sum.fitness_history);
end

function plotInWS()
    global optimLog inputData
    axis([-1,1,-1,1,-1,1])
    ni=length(optimLog.group(1).fitness_history)/optimLog.round_num;
    ng=optimLog.group_num;
    for i=1:2*ni*optimLog.round_num
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'k')
        hold on
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'rx')
        plot3(optimLog.path_history(1,1,1,i),optimLog.path_history(2,1,1,i)...
            ,optimLog.path_history(3,1,1,i),'ok');
        plot3(optimLog.path_history(1,end,end,i),optimLog.path_history(2,end,end,i)...
            ,optimLog.path_history(3,end,end,i),'dk');
        for j=1:ng
            plot3(optimLog.path_history(1,:,j,i),optimLog.path_history(2,:,j,i)...
                ,optimLog.path_history(3,:,j,i),'r')
            plot3(optimLog.regPath_history(1,:,j,i),optimLog.regPath_history(2,:,j,i)...
                ,optimLog.regPath_history(3,:,j,i),'gx')
        end
        text(-1,-1,['iteration times: ', num2str(i)],'VerticalAlignment','top','FontSize',12);
        hold off
        axis([-1,1,-1,1,-1,1])
        if i==1
            disp('');
        end
        pause(0.1)
    end
end

function plotInCS(opt)
    global optimLog inputData fitnessFun outputData
    assert(opt=='w'||opt=='c')
    ni=length(optimLog.group(1).fitness_history)/optimLog.round_num;
    ng=optimLog.group_num;
    ppg = fitnessFun.spacenum+1;
    if opt=='w'
        targetPath = regular_path(inputData.path, fitnessFun.spacenum*ng);
        path_history = optimLog.workPath_history;
        ylimits=[-1, 1];
    elseif opt=='c'
        targetPath = regular_path(outputData.junctionPos, fitnessFun.spacenum*ng);
        path_history = optimLog.jointPath_history;
        ylimits=[-pi, pi];
    end
    for i=1:2*ni*optimLog.round_num
        plot(targetPath','k+')
        hold on
        myplot([],targetPath')
        for j=1:ng
            if j==1
                myplot(1:ppg,path_history(:,:,j,i)')
                plot(1:ppg-1,path_history(:,1:end-1,j,i)','rx')
                plot(ppg,path_history(:,end,j,i)','d')
            else
                myplot(ppg*(j-1)+2-j:ppg*j+1-j,path_history(:,:,j,i)')
                plot((ppg-1)*(j-1)+2:(ppg-1)*j,path_history(:,2:end-1,j,i)','rx')
                plot((ppg-1)*j+1,path_history(:,end,j,i)','d')
            end
            axis([1,fitnessFun.spacenum*ng+1,ylimits])
        end
        text(0,-0.8,['iteration times: ', num2str(i)],'VerticalAlignment','top','FontSize',12);
        hold off
        pause(0.1)
    end
    
    function myplot(X,Y)
        % requires 'hold on'
        color_define=[0   1   0  ; 0   1   1  ; 1   1   0  ;
                      1   0   1  ; 0.5 0   0  ; 0   0   0.5];
        if isequal(X,[])
            X=1:size(Y,1);
        end
        for y=Y
            color = color_define(1,:);
            plot(X,y,'Color',color)
            color_define = color_define(2:end,:);
        end
    end
end

% 通过optimLog里各组的solutionhistory重新计算还原出每次迭代时的qTable
function calculateHistoy()
    global optimLog fitnessFun outputData inputData
    n = optimLog.group_num;
    nj = size(optimLog.qTable_history(1).q, 1);
    ni = length(optimLog.group(1).fitness_history)/optimLog.round_num;  %迭代次数
    fitnessFun.qTable = optimLog.qTable_history(1);
    tic
    for ii=1:optimLog.round_num
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
        fitnessFun.target_path=outputData.junctionPos(:,path_index);
        index=(ii-1)*ni+iter;
        if iter>ni
            index=index-ni;
        end
        solution=optimLog.group(group_num).solution_history(index,:);
        [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
        fitnessFun.qTable.q(:,group_num+1)=result(1:nj,fitnessFun.spacenum+1);
        fitnessFun.qTable.vq(:,group_num+1)=result(nj+1:nj*2,fitnessFun.spacenum+1);
        fitnessFun.qTable.aq(:,group_num+1)=result(nj*2+1:nj*3,fitnessFun.spacenum+1);
        ql=result(1:nj,:);
        path=[];
        for q=ql
            Trans=fitnessFun.fastForwardTrans(q);
            path=[path,Trans(1:3,4,7)];
        end
        optimLog.jointPath_history(:,:,group_num,(ii-1)*ni*2+iter)=...
            regular_path(ql(:,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
        optimLog.workPath_history(:,:,group_num,(ii-1)*ni*2+iter)=...
            regular_path(path(:,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
        if group_num+1<=n
            optimLog.jointPath_history(:,:,group_num+1,(ii-1)*ni*2+iter)=...
            regular_path(ql(:,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
            optimLog.workPath_history(:,:,group_num+1,(ii-1)*ni*2+iter)=...
                regular_path(path(:,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
        end
        cost=1/fitnessFun.evaluateTrajectory(result,solution);
        if group_num==1 && ii==1
            assert(abs(cost-optimLog.group(1).fitness_history(iter))<1e-4,...
                "%d %f  %f\n",iter,cost,optimLog.group(1).fitness_history(iter));
        end
        if group_num==1 && iter==ni   %第一轮最后一次迭代时第一段轨迹的代价值
            fitnessFun.target_path=outputData.junctionPos(:,path_index);
            [fit, cost_vec]=fitnessFun.evaluateTrajectory(result(:,1:fitnessFun.spacenum+1),solution);
            firstSegCost=1/fit-cost_vec(end);
        else
            firstSegCost=-1;
        end
    end
end