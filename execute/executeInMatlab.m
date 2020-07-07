%用于在matlab环境内部模拟机械臂的运行，并作展示
%主要使用matlab robotic toolbox实现
% 
global model outputData inputData


%使用toolbox自带的点到点轨迹
%outputData_bench.trajectory = jtraj(inputData.qStart,inputData.qFinal,20)';


%机械臂运行
% plotManipulator(model, outputData.trajectory(:,1),gca)
% hold off
plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
hold on
plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
axis equal
%plotManipulator(model, outputData.trajectory(1:model.joint_num,:),gca)
model.km.plot(outputData.trajectory(1:model.joint_num,:)','trail',{'r'},'delay',0.1)

figure,
plotJoint_Time(outputData);