% toolbox中自带的正运动学要调用对象，太慢，这里优化一个更快的版本
function T = fastForwardTrans(obj, theta)
    a=obj.DH(4,:);
    d=obj.DH(2,:);
    offset=obj.DH(1,:);
    alpha=obj.DH(3,:);
    base=obj.base;
    assert(size(theta,1)==obj.joint_num)
    theta=theta'+offset;
    T=zeros(4,4,obj.joint_num+1);
    T(:,:,1) = base; %joint1
    for i=1:obj.joint_num
        T(:,:,i+1) = T(:,:,i)*T_para(theta(i),d(i),a(i),alpha(i)); %joint2 -> end-effector
    end
    function T = T_para(theta,d,a,alpha)
        T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
           sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
           0,sin(alpha),cos(alpha),d;
           0,0,0,1];
    end
end

