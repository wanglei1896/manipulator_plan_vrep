%% ur5机械臂建模
% 使用matlab robotic toolbox建立机械臂模型
function result=model_ur5()
  mdl_ur5();
  ur5.base = transl(0,0,-0.0307); %为了与vrep中模型相符
  ur5.offset = [-pi/2, -pi/2, 0, -pi/2, 0, -pi/2]; %关节角偏移，同上
  
  result.km = ur5;
  for i=1:6
    result.shape(i).vex=[];
  end
end

