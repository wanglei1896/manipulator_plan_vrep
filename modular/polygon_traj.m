function [px,py,spacenum] = polygon_traj(polynum,spannum,lpnum)
%FORWORD_TRAJ 用于生成多角形轨迹
%   此处显示详细说明

o=[1.5,0];   %圆心坐标
r=1;    %圆半径
phi=0.2;  %初相角
t=linspace(phi,2*pi+phi,polynum+1);
[x1,y1]=pol2cart(t,r);
x=o(1)+x1;
y=o(2)+y1;
points=[x;y];   %角点集
t1=1; t2=t1+spannum;
for i=1:polynum
    px((i-1)*lpnum+1:i*lpnum)=linspace(x(t1),x(t2),lpnum);
    py((i-1)*lpnum+1:i*lpnum)=linspace(y(t1),y(t2),lpnum);
    t1=t2;
    t2=t1+spannum;
    if t1>polynum
        t1=t1-polynum;
    end
    if t2>polynum
        t2=t2-polynum;
    end
end
spacenum=lpnum*polynum;
%plot(px,py)
end

