%% ur5机械臂建模
% 使用matlab robotic toolbox建立机械臂模型
function result=model_serialLink()
  result.base = transl(0,0,0); %基座相对于世界坐标系的位姿
  result.DH=[];
  result.joint_num=1;
  result.shape(1).vex=[];
  result.km=[];
end

