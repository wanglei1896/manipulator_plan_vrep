global optimLog fitnessFun inputData outputData
fitnessFun.serial_number=3;
path_index=equalDivide(inputData.spacenum,optimLog.group_num,fitnessFun.serial_number);
fitnessFun.target_path=outputData.junctionPos(:,path_index);
fitnessFun.fitnessf(optimLog.group(fitnessFun.serial_number).solution_history(end,:));

plot3(regPos(1,:),regPos(2,:),regPos(3,:),'gx')
hold on
plot3(regPath(1,:),regPath(2,:),regPath(3,:))
plot3(regPath(1,:),regPath(2,:),regPath(3,:),'rx')
plot3(regPos(1,:),regPos(2,:),regPos(3,:))