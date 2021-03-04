% 格式化关节变化图(关节位置、速度、加速度)
function joint_variation(info)
global outputData model
    if isequal(info,'q')
        data=outputData.trajectory(1:model.joint_num,:);
        dof=size(data,1);
        y_label='Joint Angle (rad)';
    elseif isequal(info,'v')
        data=outputData.trajectory(model.joint_num+1:model.joint_num*2,:);
        dof=size(data,1);
        y_label='Joint Velocity (rad/s)';
    elseif isequal(info,'a')
        data=outputData.trajectory(model.joint_num*2+1:model.joint_num*3,:);
        dof=size(data,1);
        y_label='Joint Acceleration (rad/s^2)';
    end
    segnum = length(outputData.segment_times);
    spacePerSeg=floor(outputData.spacenum/segnum);
    t = linspace(outputData.segment_curtimes(1),outputData.segment_curtimes(2),spacePerSeg+1);
    for i=2:segnum
        temp = linspace(outputData.segment_curtimes(i),outputData.segment_curtimes(i+1),spacePerSeg+1);
        t=[t, temp(2:end)];
    end
    plot(t, data','-');
    setLegend(dof);
    %hold on,
    grid on,
    xticks(round(outputData.segment_curtimes,2)) %显示重要指示线
    yticks(-2*pi:pi/2:2*pi) %同上
    if isequal(info,'q') && dof==6
        ylim([-pi, 2*pi])
    else
        ylim([-pi, pi])
    end
    yticklabels({'-2\pi','-1.5\pi','-\pi','-0.5\pi','0','0.5\pi','\pi','1.5\pi','2\pi'})
    ax=gca;
    ax.XLabel.String='Time (sec)';
    ax.YLabel.String=y_label;
    ax.XLim(2)=round(outputData.segment_curtimes(end),2);
end

function setLegend(dof)
    legends=[];
    for i=1:dof
        legends=[legends;['joint', num2str(i)]];
    end
    hlegend = legend(legends);
    hlegend.NumColumns = 3;
end