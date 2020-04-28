classdef fitnessFun_ap
    %FITNESSFUN_AP 此处显示有关此类的摘要
    %   沿路径运动(along path)规划的代价函数
    
    properties
        % 访问全局变量太慢，所以存在这里作为私有数据
        d; a; alpha; base; joint_num; offset; num_joints; % 机械臂的相关参数
        linkShapes; obstacles; % 机械臂连杆和障碍物的mesh shape（顶点表示, XData, YData, ZData）
        linkCentre; obsCentre;
        spacenum; % 生成轨迹段数
        qTable; % 各轨迹段端点处的参数值（关节位置、速度、加速度）
        serial_number; %目前优化的是第几段
        target_path; %目前要优化段的目标路径
        parameter_bound;
    end
    
    methods
        function obj = fitnessFun_ap(manipulator_model)
            obj.d = manipulator_model.d;
            obj.a = manipulator_model.a;
            obj.alpha = manipulator_model.alpha;
            obj.offset = manipulator_model.offset;
            obj.base = manipulator_model.base.t;
            obj.num_joints = 3;
        end
        
        function [fitness_value,cost_vec] = fitnessf(obj, parameters)
            % 给优化算法回调用，注意接口与其保持一致
            [status, result] = obj.convertSolutionToTrajectory(parameters);
            if status ~= 0
                fitness_value = 1/(result*1000);
                return;
            end
            [fitness_value,cost_vec] = obj.evaluateTrajectory(result,parameters);
        end
        function [evaluate_value,cost_vec] = evaluateTrajectory(obj,result,parameters)
            ql=result(1:obj.num_joints,:);
            vl=result(obj.num_joints+1:obj.num_joints*2,:);
            al=result(obj.num_joints*2+1:obj.num_joints*3,:);
            n_tj=size(ql,2); %整条轨迹上的采样点
            pos=zeros(3,n_tj); %存储整条轨迹上机械臂末端位置
            %{
              ft表示轨迹中速度/加速度超过max的轨迹片段的各采样点速度/加速度之和。
              此指标可发现轨迹中速度/加速度过高的片段，并引导agent减少或改善这些片段；
              此指标对低于max的轨迹片段无指导，适合作为惩罚项；
              此指标易引发运动时间(t1、t2)的扩张，最好配合'time'指标使用
            %}
            ft=0;
            maxv=pi;
            for v=abs(vl)-maxv
                for vs=v'
                    if vs > 0
                        ft=ft+vs;
                    end
                end
            end
            %{
              fq
            %}
            fq=sum(sum(abs(diff(ql'))));
            %{
              oa表示避障指标
            %}
            collision_count=0; %用于统计轨迹上机械臂与障碍物的碰撞次数
            for i=1:n_tj
                theta=[ql(:,i);0;0;0];
                trans=fastForwardTrans(obj,theta); %forwardTrans to get the transform matrix
                for j=1:obj.num_joints
                    tran = trans(:,:,j+1);
                    centre1=tran(1:3,1:3)*obj.linkCentre(:,j)+tran(1:3,4);
                    for k=1:length(obj.obstacles)
                        centre_dis=norm(centre1-obj.obsCentre(:,k),2);
                        if centre_dis<0.1
                            collision_count=collision_count+1;
                        elseif false
                            % Do collision detection
                            if GJK(S1Obj,obj.obstacles(k),6)
                                vertices = tran(1:3,1:3)*obj.linkShapes(j).vex+tran(1:3,4);
                                S1Obj.XData=vertices(1,:)';
                                S1Obj.YData=vertices(2,:)';
                                S1Obj.ZData=vertices(3,:)';
                                collision_count=collision_count+1;
                            end
                        end
                    end
                end
                pos(:,i)=trans(1:3,4,obj.num_joints+1);
            end
            oa=collision_count;
            %{
              fdt表示机械臂末端划过的路径与给定路径的相符程度度量（越小越好）
            %}
            assert(mod(obj.spacenum,2)==0)
            if size(ql,2)==obj.spacenum+1 %区分是两段还是一段
                sp=obj.spacenum;
            else
                sp=obj.spacenum*2;
            end
            regPath = regular_path(obj.target_path, sp);
            regPos_1 = regular_path(pos(:,1:floor(size(ql,2)/2)),sp/2);
            regPos_2 = regular_path(pos(:,floor(size(ql,2)/2):end),sp/2); %相应分成两部分
            regPos = [regPos_1, regPos_2(:,2:end)];
            deltas=abs(regPath-regPos);
            Pos_punishment=[];
            fdt=norm(deltas(:,1));
            for delta=deltas(:,2:end)
                Pos_punishment=[Pos_punishment,norm(delta)];
                if sum(delta)>fdt
                    fdt=norm(delta);
                end
            end
            fdt=sum(sum(deltas));
            %{
              fdis表示机械臂末端划过的轨迹长度
            %}
            dx=diff(pos(1,:)); dy=diff(pos(2,:)); dz=diff(pos(3,:));
            dis=sqrt(dx.^2+dy.^2+dz.^2);
            fdis=sum(dis);
            %{
              time表示轨迹运动的时间
            %}
            time=sum(parameters(end));
            cost_vec=[ft,fq,fdt,oa,fdis,time, Pos_punishment];
            cost=cost_vec*[1,0,1,1,0,0, zeros(1,length(Pos_punishment))]';
            evaluate_value=1/(cost+eps); %防止/0错误
        end
        function T = fastForwardTrans(obj, theta)
            a=obj.a; d=obj.d; alpha=obj.alpha; 
            base=eye(4); base(1:3,4)=obj.base;
            theta=theta'+obj.offset;
            % toolbox中自带的正运动学要调用对象，太慢，这里优化一个更快的版本
            T=zeros(4,4,7);
            T(:,:,1) = base; %joint1
            T01=base*T_para(theta(1),d(1),a(1),alpha(1));
            T(:,:,2) = T01; %joint2
            T12=T_para(theta(2),d(2),a(2),alpha(2));
            T02=T01*T12;
            T(:,:,3) = T02; %joint3
            T23=T_para(theta(3),d(3),a(3),alpha(3));
            T03=T02*T23;
            T(:,:,4) = T03; %joint4
            T34=T_para(theta(4),d(4),a(4),alpha(4));
            T04=T03*T34;
            T(:,:,5) = T04; %joint5
            T45=T_para(theta(5),d(5),a(5),alpha(5));
            T05=T04*T45;
            T(:,:,6) = T05; %joint6
            T56=T_para(theta(6),d(6),a(6),alpha(6));
            T06=T05*T56;
            T(:,:,7) = T06; %end-effctor

            function T = T_para(theta,d,a,alpha)
                T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
                   sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
                   0,sin(alpha),cos(alpha),d;
                   0,0,0,1];
            end
        end
        
        function [status, result] = convertSolutionToTrajectory(obj, parameters)
            status = 0;
            spacenum = obj.spacenum;
            qTable = obj.qTable;
            Serialnumber = obj.serial_number;

            num_joints=obj.num_joints;

            % 左侧端点从qTable中找
            x0=qTable.q(:,Serialnumber);
            vx0=qTable.vq(:,Serialnumber);
            ax0=qTable.aq(:,Serialnumber);

            % 右侧端点来自优化向量
            x1=parameters(1:num_joints)';
            vx1=parameters(num_joints+1:num_joints*2)';
            t1=parameters(end)/2;
            t2=parameters(end)/2;
            assert(t1>0);
            
            % 右右侧端点
            if Serialnumber+2>size(qTable.q,2) %处理末端
                x2=x1;
                vx2=vx1;
                ax2=zeros(num_joints,1);
            else
                x2=qTable.q(:,Serialnumber+2);
                vx2=qTable.vq(:,Serialnumber+2);
                ax2=qTable.aq(:,Serialnumber+2);
            end

            assert(isequal(size(x0),[num_joints,1])); % 需为列向量
            assert(isequal(size(x1),[num_joints,1])); % 需为列向量
            assert(isequal(size(vx1),[num_joints,1])); % 需为列向量

            % compute interpolate factor(analytical solution)
            a00=x0;
            a01=vx0;
            a02=ax0/2;
            a03=(4*x1-vx1*t1-4*x0-3*vx0*t1-ax0*t1^2)/t1^3;
            a04=(vx1*t1-3*x1+3*x0+2*vx0*t1+ax0*t1^2/2)/t1^4;
            ax1=2*a02+6*a03*t1+12*a04*t1^2;
            A1 = [a00, a01, a02, a03, a04];

            b10=x1;
            b11=vx1;
            b12=ax1/2;
            b13=(20*x2-20*x1-(8*vx2+12*vx1)*t2-(3*ax1-ax2)*t2^2)/(2*t2^3);
            b14=(30*x1-30*x2+(14*vx2+16*vx1)*t2+(3*ax1-2*ax2)*t2^2)/(2*t2^4);
            b15=(12*x2-12*x1-(6*vx2+6*vx1)*t2-(ax1-ax2)*t2^2)/(2*t2^5);
            A2 = [b10, b11, b12, b13, b14, b15];

            spacenum1=spacenum;
            spacenum2=spacenum;
            tl1=linspace(0,t1,spacenum1+1);
            tl2=linspace(0,t2,spacenum2+1);
            tl2=tl2(2:end); % 不重复计算同一点
            
            z1=zeros(1,length(tl1));
            z2=zeros(1,length(tl2));
            result=[A1*[tl1.^0; tl1.^1; tl1.^2; tl1.^3; tl1.^4],...
                    A2*[tl2.^0; tl2.^1; tl2.^2; tl2.^3; tl2.^4; tl2.^5]; % the angle varies of each joint
                    A1*[z1; tl1.^0; 2*tl1.^1; 3*tl1.^2; 4*tl1.^3],...
                    A2*[z2; tl2.^0; 2*tl2.^1; 3*tl2.^2; 4*tl2.^3; 5*tl2.^4]; % the velocity varies of each joint
                    A1*[z1; z1; 2*tl1.^0; 6*tl1.^1; 12*tl1.^2],...
                    A2*[z2; z2; 2*tl2.^0; 6*tl2.^1; 12*tl2.^2; 20*tl2.^3]; % the acceleration varies of each joint
                   ];
            if Serialnumber+2>size(qTable.q,2) %处理末端
                result=result(:,1:spacenum+1);
            end
        end
    end
end

