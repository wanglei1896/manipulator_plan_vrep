function [px,py] = forword_traj(spacenum)
%FORWORD_TRAJ �������˶�ѧ����ĩ��ִ�����켣
%   �˴���ʾ��ϸ˵��
t1=1;   %����ʱ��
spacenum;   %ʱ��ֶ���

a0=0;
a1=1;
a2=0;
a3=0;
a4=0;
% w=[1,-1,10];
t=linspace(0,t1,spacenum);
%s�����߲�������������ò�����ʱ��ĺ�����ϵ��Ҳ���˶�����
s=a0+a1*t+a2*t.^2+a3*t.^3+a4*t.^4;
s=s*2*pi;
% x01=w'*s;
%����������ڲ���s�ļ���·��
x01=[   2*s;
        s;
        2*s];
pcart=forkin(x01,[0.6 0.6 0.3]);
% manimove_anime(x01,[0.6 0.6 0.3]);
px=pcart(1,:)+1;py=pcart(2,:);

fq=sum(sum(abs(diff(x01'))));
end

