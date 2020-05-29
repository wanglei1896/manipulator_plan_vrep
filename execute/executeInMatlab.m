%用于在matlab环境内部模拟机械臂的运行，并作展示
%主要使用matlab robotic toolbox实现
% 
global model outputData


%使用toolbox自带的点到点轨迹
%outputData_bench.trajectory = jtraj(inputData.qStart,inputData.qFinal,20)';


%机械臂运行
model.km.plot(outputData.trajectory(:,1)','trail',{'r'})
hold on
plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
model.km.plot(outputData.trajectory(:,:)','trail',{'r'});

figure,
plotJoint_Time(outputData);