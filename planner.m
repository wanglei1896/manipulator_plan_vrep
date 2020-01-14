format long;
global initialJoint outputjointValue pStart pFinal diseredPosition ur5_kinemo

q0=initialJoint+[-pi/2, -pi/2, 0, -pi/2, 0, -pi/2];%初始关节角
manipulator = ur5_kinemo;

% [xx,yy,zz]=position(q0);
% posend=[xx,yy,zz]
% Jacob(q0)
qStart = manipulator.ikunc(pStart);
qFinal = manipulator.ikunc(pFinal);

T=20;
tau=0.1;
h=0.1;
gamma=h/tau;
t=0:tau:T;
iter=length(t);
% 关节位置
theta(:,1)=q0;

%[Ppx,Ppy,Ppz]=position(q0);
initx=pStart(1,4);
inity=pStart(2,4);
initz=pStart(3,4);
% 期望位置与实际位置
diseredPosition=zeros(3,iter);
actualPosition=zeros(3,iter);
% 输出的关节运动序列
outputjointValue=zeros(6,iter);


disp('planning start.');

for k=1:iter
pt=eye(4);
diseredPosition(1,k)=0.3*sin(pi*t(k)/5);%0.1*sin(pi*t(k)/5)*cos(sin(3*pi*t(k)/10))+initx-0.01*sin(pi*0/5)*cos(sin(3*pi*0/10));
diseredPosition(2,k)=0.3*cos(pi*t(k)/5);%0.1*sin(pi*t(k)/5)*sin(sin(3*pi*t(k)/10))+inity-0.01*sin(pi*0/5)*sin(sin(3*pi*0/10));
diseredPosition(3,k)=initz+0.1*(sin(pi*t(k)));

pt(1,4)=diseredPosition(1,k);
pt(2,4)=diseredPosition(2,k);
pt(3,4)=diseredPosition(3,k);

try
    outputjointValue(:,k)=manipulator.ikunc(pt);
catch e
    disp(e);
    disp(diseredPosition(:,k));
end
% drdx(k)=0.1*(pi/5*cos(pi*t(k)/5)*cos(sin(3*pi*t(k)/10))-3*pi/10*cos(3*pi*t(k)/10)*sin(sin(3*pi*t(k)/10))*sin(pi*t(k)/5));
% drdy(k)=0.1*(pi/5*cos(pi*t(k)/5)*sin(sin(3*pi*t(k)/10))+3*pi/10*cos(3*pi*t(k)/10)*cos(sin(3*pi*t(k)/10))*sin(pi*t(k)/5));
% drdz(k)=0;

% [Ppx,Ppy,Ppz]=position(theta(:,k));
% actualPosition(:,k)=[Ppx(6);Ppy(6);Ppz(6)];

end

disp('planning ended');