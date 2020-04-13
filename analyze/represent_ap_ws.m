%% along path规划中优化过程的图形化展示
% 优化过程中各个解代表的机械臂末端路径的变化
% 

if isequal(optimLog.path_history,[])
    calculateHistoy();
end

plotOthers();
%figure,
%plotInWS();
%figure,
%plotInCS();

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
    ni=length(optimLog.group(1).fitness_history);
    ng=optimLog.group_num;
    for i=1:(2*ni)
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'k')
        hold on
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'rx')
        plot3(optimLog.path_history(1,1,1,i),optimLog.path_history(2,1,1,i)...
            ,optimLog.path_history(3,1,1,i),'ok');
        plot3(optimLog.path_history(1,end,end,i),optimLog.path_history(2,end,end,i)...
            ,optimLog.path_history(3,end,end,i),'dk');
        for j=1:ng
            if mod(j,2)==1
                if i<=ni
                    path_color='r';
                else
                    path_color='b';
                end
            elseif i>ni
                path_color='r';
            else
                path_color='b';
            end
            plot3(optimLog.path_history(1,:,j,i),optimLog.path_history(2,:,j,i)...
                ,optimLog.path_history(3,:,j,i),path_color)
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
    ni=length(optimLog.group(1).fitness_history);
    ng=optimLog.group_num;
    ppg = outputData.spacenum/optimLog.group_num+1;
    targetPath = regular_path(inputData.path, outputData.spacenum);
    for i=1:(2*ni)
        plot(targetPath','k+')
        hold on
        myplot([],targetPath')
        for j=1:ng
            if mod(j,2)==1
                if i<=ni
                    path_color='r';
                else
                    path_color='b';
                end
            elseif i>ni
                path_color='r';
            else
                path_color='b';
            end
            if j==1
                myplot(1:ppg,optimLog.regPath_history(:,:,j,i)')
                plot(1:ppg-1,optimLog.regPath_history(:,1:end-1,j,i)',[path_color, 'x'])
                plot(ppg,optimLog.regPath_history(:,end,j,i)','d')
            else
                myplot(ppg*(j-1)+2-j:ppg*j+1-j,optimLog.regPath_history(:,:,j,i)')
                plot((ppg-1)*(j-1)+2:(ppg-1)*j,optimLog.regPath_history(:,2:end-1,j,i)',[path_color, 'x'])
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
    ni = length(optimLog.group(1).fitness_history);  %迭代次数
    qTable_initial = optimLog.qTable_history(1);
    spacePerGroup = ceil(inputData.spacenum/n);
    remain = mod(inputData.spacenum,n);
    tic
    for i=1:ni
        fitnessFun.qTable=qTable_initial;
        sum=0;
        for j=1:n
            fitnessFun.serial_number=j;
            if j==n && remain>0
                path_index = (inputData.spacenum-remain+1):inputData.spacenum;
            else
                path_index = (1:(spacePerGroup+1))+(j-1)*spacePerGroup;
            end
            fitnessFun.target_path=inputData.path(:,path_index);
            if mod(j,2)==1
                solution=optimLog.group(j).solution_history(i,:);
                [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
                fitnessFun.qTable.q(:,j+1)=result(1:nj,end);
                fitnessFun.qTable.vq(:,j+1)=result(nj+1:nj*2,end);
                fitnessFun.qTable.aq(:,j+1)=result(nj*2+1:nj*3,end);
            else
                solution=[qTable_initial.q(:,j+1)',qTable_initial.vq(:,j+1)',qTable_initial.aq(:,j+1)',10]; %时间默认为10
                [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
            end
            path=result(1:nj,:);
            optimLog.path_history(:,:,j,i)=path;
            optimLog.regPath_history(:,:,j,i)=regular_path(path,size(path,2)-1);
            sum=sum+1/fitnessFun.evaluateTrajectory(result,solution);
            if j==1
                assert(abs(sum-optimLog.group(1).fitness_history(i))<0.0001,...
                    "%f  %f\n",sum,optimLog.group(1).fitness_history(i));
            end
        end
        optimLog.qTable_history(i+1)=fitnessFun.qTable;
        optimLog.sum.fitness_history(i)=sum;
    end
    qTable_initial = optimLog.qTable_history(ni+1);
    for i=(ni+1):(2*ni)
        fitnessFun.qTable=qTable_initial;
        sum=0;
        for j=1:n
            fitnessFun.serial_number=j;
            if j==n && remain>0
                path_index = (inputData.spacenum-remain+1):inputData.spacenum;
            else
                path_index = (1:(spacePerGroup+1))+(j-1)*spacePerGroup;
            end
            fitnessFun.target_path=inputData.path(:,path_index);
            if mod(j,2)==0
                solution=optimLog.group(j).solution_history(i-ni,:);
                [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
                fitnessFun.qTable.q(:,j+1)=result(1:nj,end);
                fitnessFun.qTable.vq(:,j+1)=result(nj+1:nj*2,end);
                fitnessFun.qTable.aq(:,j+1)=result(nj*2+1:nj*3,end);
            else
                solution=optimLog.group(j).solution_history(end,:); %前一轮的最后一次迭代
                [~,result]=fitnessFun.convertSolutionToTrajectory(solution);
            end
            path=result(1:nj,:);
            optimLog.path_history(:,:,j,i)=path;
            optimLog.regPath_history(:,:,j,i)=regular_path(path,size(path,2)-1);
            sum=sum+1/fitnessFun.evaluateTrajectory(result,solution);
            if j==1
                assert(abs(sum-optimLog.group(1).fitness_history(end))<0.0001,...
                    "%f  %f\n",sum,optimLog.group(1).fitness_history(end));
            end
        end
        optimLog.qTable_history(i+1)=fitnessFun.qTable;
        optimLog.sum.fitness_history(i)=sum;
    end
    toc
    endPath=optimLog.path_history(:,1,1,end);
    for i=1:n
        path=optimLog.path_history(:,:,i,end);
        endPath=[endPath,path(:,2:end)];
    end
    outputData.endPath=endPath;
end