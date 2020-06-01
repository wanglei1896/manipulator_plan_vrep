% 用于绘制机械臂

function plotManipulator(manipulator, thetas, axe)
    % Make a figure
    if nargin<=2
        fig=figure;
        axis equal
        axe=fig.Children;
    end
    hold(axe,'on')

    % Make shape 1 (links)
    S1Objs=[]; S1Coords=[];
    for link=manipulator.shape
        S1.Vertices = link.vex';
        S1.Faces = link.face'+1;
        S1.FaceVertexCData = jet(size(S1.Vertices,1));
        S1.FaceColor = 'interp';
        S1Objs = [S1Objs, patch(axe,S1)];
        S1Coords = [S1Coords, S1];
    end
    
    hold(axe,'off')
    %axe.Visible = 'off'; % Turn off the axis for more pleasant viewing.
    axe.Color = [1 1 1];
    rotate3d(axe,'on');
    
    for j=1:size(thetas,2)
        trans=fastForwardTrans(thetas(:,j)); %forwardTrans to get the transform matrix
        for i=1:6
            tran = trans(:,:,i+1);
            S1Objs(i).Vertices = (tran(1:3,1:3)*S1Coords(i).Vertices'+tran(1:3,4))';
        end
        pause(0.1)
        drawnow;
    end
    function pos = fastForwardTrans(theta)
        a=manipulator.DH(4,:); d=manipulator.DH(2,:); alpha=manipulator.DH(3,:); 
        offset = manipulator.DH(1,:);
        base=manipulator.base;
        theta=theta'+offset;
        % toolbox中自带的正运动学要调用对象，太慢，这里优化一个更快的版本
        pos=zeros(4,4,7);
        pos(:,:,1) = base; %joint1
        T01=base*T_para(theta(1),d(1),a(1),alpha(1));
        pos(:,:,2) = T01; %joint2
        T12=T_para(theta(2),d(2),a(2),alpha(2));
        T02=T01*T12;
        pos(:,:,3) = T02; %joint3
        T23=T_para(theta(3),d(3),a(3),alpha(3));
        T03=T02*T23;
        pos(:,:,4) = T03; %joint4
        T34=T_para(theta(4),d(4),a(4),alpha(4));
        T04=T03*T34;
        pos(:,:,5) = T04; %joint5
        T45=T_para(theta(5),d(5),a(5),alpha(5));
        T05=T04*T45;
        pos(:,:,6) = T05; %joint6
        T56=T_para(theta(6),d(6),a(6),alpha(6));
        T06=T05*T56;
        pos(:,:,7) = T06; %end-effctor
        %positions=[[0 0 0]', T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4), T05(1:3,4), T06(1:3,4)];
        %pos = positions(:,number+1);
    end
    function T = T_para(theta,d,a,alpha)
        T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
           sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
           0,sin(alpha),cos(alpha),d;
           0,0,0,1];
    end
end

