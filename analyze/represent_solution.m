%% ����ͼ�λ�չʾ�Ż��㷨���Ż�����
global optimLog

if ~exist('histo_t1_t2','var')
    disp('��ʼ����'), tic,
    histo_t1_t2 = calHisto_t1_t2();
    toc, disp('�������')
end

% ����ֵ����������仯
plot(optimLog.fitness_history);

figure, 
plothisto_t1_t2(histo_t1_t2,optimLog.solution_history(:,end-1:end));

% t1-t2 �������Ӧ�Ⱥ�������������仯(����ά�ȵ�ֵΪ�������Ž��ֵ)
function plothisto_t1_t2(histo, actual_pos)
    %xlim([0,10]); ylim([0,10])
    global optimLog
    ylabel t1
    xlabel t2
   
    %p = plot(actual_pos(1,2)*30/10,actual_pos(1,1)*30/10,1,'o','MarkerFaceColor','red');
    h = surf(histo.t1, histo.t2, histo.cost(:,:,1));
    t = text(0,0,'','VerticalAlignment','top','FontSize',12);
    for i=2:length(optimLog.fitness_history)
        hold on
        %p.XData = actual_pos(i,2)*30/10; %X����Ϊt2
        %p.YData = actual_pos(i,1)*30/10; %Y����Ϊt1
        t.String = ['���������� ', num2str(i)];
        h.ZData = histo.cost(:,:,i);
        colorbar;
        axis manual
        hold off
        drawnow limitrate
    end
end

% ���㺯����t1-t2������ȵ�ͼ,��Ҫ�õ����ۺ���
function result = calHisto_t1_t2()
    global optimLog fitnessFun
    dividenum = 30;
    t1=linspace(0,10,dividenum+2); t2=linspace(0,10,dividenum+2);
    t1=t1(2:end-1); t2=t2(2:end-1);
    result.t1 = t1;  result.t2 = t2;
    result.cost = zeros(dividenum,dividenum,length(optimLog.fitness_history));
    for i=1:length(optimLog.fitness_history)
        sol=optimLog.solution_history(i,:);
        otherD = sol(1:12);
        for j = 1:dividenum
            for k = 1:dividenum
                result.cost(j,k,i) = 1/fitnessFun.fitnessf([otherD,t1(j),t2(k)]);
            end
        end
    end
end