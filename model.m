%% 机械臂建模
% 使用DH参数建立机械臂模型
function [ur5_kinemo]=model()
  mdl_ur5();
  ur5.base = transl(0,0,-0.0307);
  ur5_kinemo = ur5;
end

