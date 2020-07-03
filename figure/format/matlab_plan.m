% 格式化matlab端规划过程图
plot3(inputData.obstacles(1).vex(1,:),inputData.obstacles(1).vex(2,:),inputData.obstacles(1).vex(3,:))
hold on
plot3(inputData.obstacles(2).vex(1,:),inputData.obstacles(2).vex(2,:),inputData.obstacles(2).vex(3,:))
plot3(inputData.obstacles(3).vex(1,:),inputData.obstacles(3).vex(2,:),inputData.obstacles(3).vex(3,:))
plot3(inputData.path(1,:),inputData.path(2,:),inputData.path(3,:))
axis equal
model.km.plot(outputData.trajectory(1:model.joint_num,:)','trail',{'r'},'delay',0.1);