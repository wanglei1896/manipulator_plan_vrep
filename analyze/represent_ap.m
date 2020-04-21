%% along path规划中优化过程的图形化展示
% 优化过程中各个解代表的机械臂末端路径的变化
%

if isequal(optimLog.path_history,[])
    calculateHistoy();
end

plotOthers();
figure,
%plotInWS();
%figure,
plotInCS();

function plotOthers()
global optimLog
	plot([optimLog.group(1).fitness_history, optimLog.group(2).fitness_history])
    legend group1 group2
    figure,
    plot(optimLog.group(1).fitvec_history(:,6:end));
    figure,
    plot(optimLog.group(2).fitvec_history(:,6:end));
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

function plotInCS()
    global optimLog inputData outputData
    ni=length(optimLog.group(1).fitness_history)/optimLog.round_num;
    ng=optimLog.group_num;
    ppg = outputData.spacenum/optimLog.group_num+1;
    targetPath = regular_path(inputData.path, outputData.spacenum);
    for i=1:2*ni*optimLog.round_num
        plot(targetPath','k+')
        hold on
        myplot([],targetPath')
        for j=1:ng
            if j==1
                myplot(1:ppg,optimLog.regPath_history(:,:,j,i)')
                plot(1:ppg-1,optimLog.regPath_history(:,1:end-1,j,i)','rx')
                plot(ppg,optimLog.regPath_history(:,end,j,i)','d')
            else
                myplot(ppg*(j-1)+2-j:ppg*j+1-j,optimLog.regPath_history(:,:,j,i)')
                plot((ppg-1)*(j-1)+2:(ppg-1)*j,optimLog.regPath_history(:,2:end-1,j,i)','rx')
                plot((ppg-1)*j+1,optimLog.regPath_history(:,end,j,i)','d')
            end
            axis([1,outputData.spacenum+1,-1,1])
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
            fitnessFun.serial_number=j;
            path_index=equalDivide(inputData.spacenum,n,j);
            fitnessFun.target_path=inputData.path(:,path_index);
            solution=optimLog.group(j).solution_history((ii-1)*ni+i,:);
            [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
            fitnessFun.qTable.q(:,j+1)=result(1:nj,fitnessFun.spacenum+1);
            fitnessFun.qTable.vq(:,j+1)=result(nj+1:nj*2,fitnessFun.spacenum+1);
            fitnessFun.qTable.aq(:,j+1)=result(nj*2+1:nj*3,fitnessFun.spacenum+1);
            ql=result(1:nj,:);
            path=[];
            for q=ql
                Trans=fitnessFun.fastForwardTrans(q);
                path=[path,Trans(1:3,4,7)];
            end
            optimLog.path_history(:,:,j,(ii-1)*ni*2+i)=path(:,1:fitnessFun.spacenum+1);
            optimLog.regPath_history(:,:,j,(ii-1)*ni*2+i)=...
                regular_path(path(:,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
            if j+1<=n
                optimLog.path_history(:,:,j+1,(ii-1)*ni*2+i)=path(:,fitnessFun.spacenum+1:end);
                optimLog.regPath_history(:,:,j+1,(ii-1)*ni*2+i)=...
                    regular_path(path(:,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
            end
            sum=sum+1/fitnessFun.evaluateTrajectory(result,solution);
            if j==1 && ii==1
                assert(abs(sum-optimLog.group(1).fitness_history(i))<1e-4,...
                	"%d %f  %f\n",i,sum,optimLog.group(1).fitness_history(i));
            end
            if j==1 && i==ni   %第一轮最后一次迭代时第一段轨迹的代价值
                fitnessFun.target_path=inputData.path(:,path_index(1:floor(length(path_index)/2+1)));
                [fit, cost_vec]=fitnessFun.evaluateTrajectory(result(:,1:fitnessFun.spacenum+1),solution);
                firstSegCost=1/fit-cost_vec(end);
            end
        end
        optimLog.qTable_history((ii-1)*ni*2+i+1)=fitnessFun.qTable;
        optimLog.sum.fitness_history((ii-1)*ni*2+i)=sum;
    end
    for i=(ni+1):(2*ni)
        optimLog.path_history(:,:,1,(ii-1)*ni*2+i)=optimLog.path_history(:,:,1,(ii-1)*ni*2+ni);
        optimLog.regPath_history(:,:,1,(ii-1)*ni*2+i)=optimLog.regPath_history(:,:,1,(ii-1)*ni*2+ni);
        sum=firstSegCost;  %第一段轨迹的代价值
        for j=2:2:n
            fitnessFun.serial_number=j;
            path_index=equalDivide(inputData.spacenum,n,j);
            fitnessFun.target_path=inputData.path(:,path_index);
            solution=optimLog.group(j).solution_history((ii-1)*ni+i-ni,:);
            [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
            fitnessFun.qTable.q(:,j+1)=result(1:nj,fitnessFun.spacenum+1);
            fitnessFun.qTable.vq(:,j+1)=result(nj+1:nj*2,fitnessFun.spacenum+1);
            fitnessFun.qTable.aq(:,j+1)=result(nj*2+1:nj*3,fitnessFun.spacenum+1);
            ql=result(1:nj,:);
            path=[];
            for q=ql
                Trans=fitnessFun.fastForwardTrans(q);
                path=[path,Trans(1:3,4,7)];
            end
            regPath=regular_path(path,size(path,2)-1);
            optimLog.path_history(:,:,j,(ii-1)*ni*2+i)=path(:,1:fitnessFun.spacenum+1);
            optimLog.regPath_history(:,:,j,(ii-1)*ni*2+i)=...
                regular_path(path(:,1:fitnessFun.spacenum+1),fitnessFun.spacenum);
            if j+1<=n
                optimLog.path_history(:,:,j+1,(ii-1)*ni*2+i)=path(:,fitnessFun.spacenum+1:end);
                optimLog.regPath_history(:,:,j+1,(ii-1)*ni*2+i)=...
                    regular_path(path(:,fitnessFun.spacenum+1:end),fitnessFun.spacenum);
            end
            sum=sum+1/fitnessFun.evaluateTrajectory(result,solution);
        end
        optimLog.qTable_history((ii-1)*ni*2+i+1)=fitnessFun.qTable;
        optimLog.sum.fitness_history((ii-1)*ni*2+i)=sum;
    end
    end
    toc
    endPath=optimLog.path_history(:,1,1,end);
    for i=1:n
        path=optimLog.path_history(:,:,i,end);
        endPath=[endPath,path(:,2:end)];
    end
    outputData.endPath=endPath;
end