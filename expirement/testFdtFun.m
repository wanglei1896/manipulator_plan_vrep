global optimLog fitnessFun
fitnessFun.fitnessf(optimLog.solution_history(end,:));

plot3(regPos(1,:),regPos(2,:),regPos(3,:),'gx')
hold on
plot3(target_path(1,:),target_path(2,:),target_path(3,:))
plot3(target_path(1,:),target_path(2,:),target_path(3,:),'rx')
plot3(regPos(1,:),regPos(2,:),regPos(3,:))