-- 约定1：要控制的机械臂放在名为'manipulators'的dummy下
mhandle=sim.getObjectChild(sim.getObjectHandle('manipulators'),0)
jhandle=sim.getObjectsInTree(mhandle,sim.object_joint_type)
jointNum=#jhandle
name=sim.getObjectName(mhandle);
link_handles={-1,-1,-1,-1,-1,-1} --连杆object的handle
for i=1,jointNum,1 do
   link_handles[i]=sim.getObjectHandle(name.."_link"..(i+1))
end

-- 约定2：障碍物的模型放在名为'Obstacles'的dummy下
obstacle_handles=sim.getObjectsInTree(sim.getObjectHandle('Obstacles'), sim.object_shape_type)

-- 约定3: 要追踪的path对象命名为'targetPath',并用一个名为'target'的dummy去采样路径点
-- 注意：'targetPath'和'target'之间不能是父子
hPath = sim.getObjectHandle('targetPath');
htarget = sim.getObjectHandle('target');

