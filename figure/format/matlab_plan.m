% 格式化matlab端规划过程图
plotMatlabPlan()
function plotMatlabPlan()
global inputData outputData model
    
    plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
    hold on
    axis equal
    colorPane='rgbymkc';
    for obstacle=inputData.obstacles
        S2.Vertices = obstacle.vex';
        S2.Faces = obstacle.face'+1;
        S2.FaceVertexCData = jet(size(S2.Vertices,1));
        S2.FaceColor = 'interp';
        patch(S2)
    end
    model.km.plot(outputData.trajectory(1:model.joint_num,end)','workspace',[-0.5,0.5,-0.5,0.5,-0.2,1],'jointdiam',1,'nowrist');
    for i=1:size(outputData.trajectory,2)
        theta=outputData.trajectory(1:model.joint_num,i);
        %model.km.plot(theta');
        T=fastForwardTrans(model,theta);
        for j=1:size(T,3)-1
            line([T(1,4,j),T(1,4,j+1)],[T(2,4,j),T(2,4,j+1)],[T(3,4,j),T(3,4,j+1)],'Color',colorPane(j));hold on;
        end
    end
    
end