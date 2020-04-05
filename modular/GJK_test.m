% Example script for GJK function
%   Animates two objects on a collision course and terminates animation
%   when they hit each other. 

function GJK_test()
global inputData model outputData
    %How many iterations to allow for collision detection.
    iterationsAllowed = 6;

    % Make a figure
    fig = figure;
    hold on

    % Make shape 1 (links)
    S1Objs=[]; S1Coords=[];
    for link=model.shape
        S1.Vertices = link.vex';
        S1.Faces = link.face'+1;
        S1.FaceVertexCData = jet(size(S1.Vertices,1));
        S1.FaceColor = 'interp';
        S1Objs = [S1Objs, patch(S1)];
        S1Coords = [S1Coords, S1];
    end

    % Make shape 2 (obstacles)
    S2Objs = [];
    for obstacle=inputData.obstacles
        S2.Vertices = obstacle.vex';
        S2.Faces = obstacle.face'+1;
        S2.FaceVertexCData = jet(size(S2.Vertices,1));
        S2.FaceColor = 'interp';
        S2Objs = [S2Objs, patch(S2)];
    end

    hold off
    axis equal
    axis([-1 1 -1 1 -1 1])
    fig.Children.Visible = 'off'; % Turn off the axis for more pleasant viewing.
    fig.Color = [1 1 1];
    rotate3d on;
    
    % Random polynomial joint trajectory
    t=linspace(0,5,61);
    para=rands(6,4);
    thetas=para*[ones(1,61);t;t.^2;t.^3];
    thetas=outputData.trajectory;

    % Animation loop. Terminates on collision.
    try
        collisionFlag = false;
        collision_count = 0;
        for theta = thetas
            trans=fastForwardTrans(theta); %forwardTrans to get the transform matrix
            for i=1:6
                tran = trans(:,:,i+1);
                S1Objs(i).Vertices = (tran(1:3,1:3)*S1Coords(i).Vertices'+tran(1:3,4))';
                for S2Obj = S2Objs
                    % Do collision detection
                    if GJK(S1Objs(i),S2Obj,iterationsAllowed)
                        collisionFlag = true;
                        collision_count=collision_count+1;
                    end
                end
            end
            drawnow;
            %assert(~collisionFlag, 'collision');
        end
    catch e
        if isequal(e.message,'collision')
            text(0.3,0.3,0.3,'Collision!','FontSize',30);
        else
            disp(e.message)
            disp(['At Line: ',num2str(e.stack.line)])
        end
    end
    disp(collision_count)
    
    function pos = fastForwardTrans(theta)
        a=model.km.a; d=model.km.d; alpha=model.km.alpha; 
        base=eye(4); base(1:3,4)=model.km.base.t;
        theta=theta'+model.km.offset;
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



