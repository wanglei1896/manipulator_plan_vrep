%% ur5��е�۽�ģ
% ʹ��matlab robotic toolbox������е��ģ��
function [ur5_kinemo]=model_ur5()
  mdl_ur5();
  ur5.base = transl(0,0,-0.0307);
  ur5_kinemo = ur5;
end

