% 用于显示天牛种群在迭代过程中的运动情形
plot_group_move(1,1,2,3);

function plot_group_move(gnumber,dim1,dim2,dim3)
global optimLog
    assert(nargin<=4)
    solution_history=optimLog.group(gnumber).all_solution_history;
    for i=1:size(solution_history,1)
        % each frame
        clf
        axis([0,1,0,1])
        xlabel(num2str(dim1))
        ylabel(num2str(dim2))
        if nargin==4
            zlabel(num2str(dim3))
            axis([0,1,0,1,0,1])
        end
        hold on
        for j=1:size(solution_history,2)
            % each point
            solution=reshape(solution_history(i,j,:),1,size(solution_history,3));
            if nargin==4
                plot3(solution(dim1),solution(dim2),solution(dim3),'o')
            elseif nargin==3
                plot(solution(dim1),solution(dim2),'o')
            end
        end
        text(0,1,['iteration times: ', num2str(i)],'VerticalAlignment','top','FontSize',12);
        hold off
        pause(0.1)
    end
end