function [px,py] = forword_traj(spacenum)
%FORWORD_TRAJ 用于正运动学生成末端执行器轨迹
%   此处显示详细说明
t1=1;   %结束时间
spacenum;   %时间分段数

a0=0;
a1=1;
a2=0;
a3=0;
a4=0;
% w=[1,-1,10];
t=linspace(0,t1,spacenum);
%s是曲线参数，下面给出该参数与时间的函数关系，也叫运动法则
s=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4;
s=s*2*pi;
% x01=w'*s;
%下面给出关于参数s的几何路径
x01=[   2*s;
        s;
        2*s];
pcart=forkin(x01,[0.6 0.6 0.3]);
% manimove_anime(x01,[0.6 0.6 0.3]);
px=pcart(1,:)+1;py=pcart(2,:);

fq=sum(sum(abs(diff(x01'))));
end

