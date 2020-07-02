%% along path规划中优化过程的图形化展示
% 优化过程中各个解代表的机械臂末端路径的变化
%

if isequal(optimLog.path_history,[])
    calculateHistory();
end

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
    round_num=1;
    ni=length(optimLog.group(1).fitness_history)/round_num;
    ng=optimLog.group_num;
    for i=1:2*ni*round_num
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'k')
        hold on
        plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:),'rx')
        plot3(optimLog.workPath_history(1,1,1,i),optimLog.workPath_history(2,1,1,i)...
            ,optimLog.workPath_history(3,1,1,i),'ok');
        plot3(optimLog.workPath_history(1,end,end,i),optimLog.workPath_history(2,end,end,i)...
            ,optimLog.workPath_history(3,end,end,i),'dk');
        for j=1:ng
            plot3(optimLog.workPath_history(1,:,j,i),optimLog.workPath_history(2,:,j,i)...
                ,optimLog.workPath_history(3,:,j,i),'r')
            plot3(optimLog.workPath_history(1,:,j,i),optimLog.workPath_history(2,:,j,i)...
                ,optimLog.workPath_history(3,:,j,i),'gx')
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
    round_num=1;
    ni=length(optimLog.group(1).fitness_history)/round_num;
    ng=optimLog.group_num;
    ppg = fitnessFun.spacenum+1;
    if opt=='w'
        targetPath = regular_path(inputData.path, fitnessFun.spacenum*ng);
        path_history = optimLog.workPath_history;
        ylimits=[-1, 1];
    elseif opt=='c'
        targetPath = outputData.jointPath;
        path_history = optimLog.jointPath_history;
        ylimits=[-pi, pi];
    end
    for i=1:2*ni*round_num
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
