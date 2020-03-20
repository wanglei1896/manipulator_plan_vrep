% 任务空间位姿插值，然后逐个做逆运动学变换
% 主要用来观察严格按照任务空间路径走的话，关节的变化情况

plot(genTraj()')

function jtraj = genTraj()
    global model
    spacenum=20;
    sx=linspace(0.3,0.45,spacenum+1);
    sy=linspace(0.4,0.2,spacenum+1);
    sz=linspace(0,0,spacenum+1);
    traj=zeros(4,4,spacenum+1);
    jtraj=zeros(6,spacenum+1);
    for i=1:20
        traj(:,:,i)=eye(4);
        traj(1:3,4,i)=[sx(i);sy(i);sz(i)];
        jtraj(:,i)=model.ikine(traj(:,:,i));
    end
    disp([sx(17),sy(17),sz(17)])
    disp(jtraj(:,17))
    disp([sx(18),sy(18),sz(18)])
    disp(jtraj(:,18))
end
