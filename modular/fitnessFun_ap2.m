classdef fitnessFun_ap2
    %FITNESSFUN_AP 此处显示有关此类的摘要
    %   沿路径运动(along path)规划的代价函数
    
    properties
        % 访问全局变量太慢，所以存在这里作为私有数据
        d; a; alpha; base; joint_num; offset; % 机械臂的相关参数
        linkShapes; obstacles; % 机械臂连杆和障碍物的mesh shape（顶点表示）
        linkCentre; obsCentre;
        spacenum; % 生成轨迹段数
        qTable; % 各轨迹段端点处的参数值（关节位置、速度、加速度）
        serial_number; %目前优化的是第几段
        target_path; %目前要优化段的目标路径(关节空间)
        parameter_bound;
        hyperparameter;
    end
    
    methods
        function obj = fitnessFun_ap2(manipulator_model)
            assert(size(manipulator_model.DH,1)==4)
            obj.offset = manipulator_model.DH(1,:);
            obj.d = manipulator_model.DH(2,:);
            obj.alpha = manipulator_model.DH(3,:);
            obj.a = manipulator_model.DH(4,:);
            obj.base = manipulator_model.base;
            obj.joint_num = manipulator_model.joint_num;
            obj.linkShapes = manipulator_model.shape;
        end
        
        function [fitness_value,cost_vec] = fitnessf(obj, parameters)
            % 给优化算法回调用，注意接口与其保持一致
            result = obj.convertSolutionToTrajectory(parameters);
            [fitness_value,cost_vec] = obj.evaluateTrajectory(result,parameters);
        end
        function [evaluate_value,cost_vec] = evaluateTrajectory(obj,result,parameters)
            ql=result(1:obj.joint_num,:);
            vl=result(obj.joint_num+1:2*obj.joint_num,:);
            al=result(2*obj.joint_num:3*obj.joint_num,:);
            n_tj=size(ql,2); %整条轨迹上的采样点
            pos=zeros(3,n_tj); %末端轨迹
            %{
              ft表示轨迹中速度/加速度超过max的轨迹片段的各采样点速度/加速度之和。
              此指标可发现轨迹中速度/加速度过高的片段，并引导agent减少或改善这些片段；
              此指标对低于max的轨迹片段无指导，适合作为惩罚项；
              此指标易引发运动时间(t1、t2)的扩张，最好配合'time'指标使用
            %}
            ft=0;
            maxv=obj.hyperparameter.ap2_maxv;
            maxa=obj.hyperparameter.ap2_maxa;
            for ft_i=1:obj.joint_num
                for ft_j=1:n_tj
                    ex_v=abs(vl(ft_i,ft_j))-maxv;
                    ex_a=abs(al(ft_i,ft_j))-maxa;
                    if ex_v>0
                        ft=ft+ex_v;
                    end
                    if ex_a>0
                        ft=ft+ex_a;
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
            oa=0;
            if obj.hyperparameter.ap2_obflag==true
            for i=1:n_tj
                theta=ql(:,i);
                min_dis=1;
                trans=fastForwardTrans(obj,theta); %forwardTrans to get the transform matrix
                for j=1:obj.joint_num
                    tran = trans(:,:,j+1);
                    for k=1:length(obj.obstacles)
                        vertices = tran(1:3,1:3)*obj.linkShapes(j).vex+tran(1:3,4);
                        % Do collision detection
                        dis=openGJK(vertices,obj.obstacles(k).vex);
                        if dis<min_dis
                            min_dis=dis;
                        end
                    end
                end
                if min_dis>obj.hyperparameter.ob_e
                    ot=1/min_dis;
                else% min_dis<=1e-4
                    ot=1/obj.hyperparameter.ob_e;
                end
                ot=ot*obj.hyperparameter.ob_e;
                oa=oa+ot;
                pos(:,i)=trans(1:3,4,7);
            end
            end
            %{
              fdt表示机械臂末端划过的路径与给定路径的相符程度度量（越小越好）
            %}
            %区分是两段还是一段
            if n_tj==obj.spacenum+1  %一段
                regPos = regular_path(ql, obj.spacenum);   
            else  %两段
                regPos_1 = regular_path(ql(:,1:obj.spacenum+1),obj.spacenum);
                regPos_2 = regular_path(ql(:,obj.spacenum+1:end),obj.spacenum); %相应分成两部分
                regPos = [regPos_1, regPos_2(:,2:end)];
            end
            regTarget = regular_path(obj.target_path(1:obj.joint_num,:), n_tj-1);
            deltas=abs(regTarget-regPos);
            Pos_punishment=[];
            fdt=norm(deltas(:,1));
            for delta=deltas(:,2:end)
                Pos_punishment=[Pos_punishment,norm(delta)];
                fdt=fdt+norm(delta);
                %if norm(delta)>fdt
                %    fdt=norm(delta);
                %end
            end
            %{
              fdis表示机械臂末端划过的轨迹长度
            %}
            fdis=0;
            if obj.hyperparameter.ap2_obflag==true
                dx=diff(pos(1,:)); dy=diff(pos(2,:)); dz=diff(pos(3,:));
                dis=sqrt(dx.^2+dy.^2+dz.^2);
                fdis=sum(dis);
            end
            %{
              time表示轨迹运动的时间
            %}
            time=sum(parameters(end));
            cost_vec=[ft,fq,fdt,oa,fdis,time, Pos_punishment];
            cost=cost_vec*[obj.hyperparameter.ap2_tradeoff, zeros(1,length(Pos_punishment))]';
            evaluate_value=1/(cost+eps); %防止/0错误
        end
        function T = fastForwardTrans(obj, theta)
            a=obj.a; d=obj.d; alpha=obj.alpha; 
            base=obj.base;
            theta=theta'+obj.offset;
            % toolbox中自带的正运动学要调用对象，太慢，这里优化一个更快的版本
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
        
        function result = convertSolutionToTrajectory(obj, parameters)
            spacenum = obj.spacenum;
            qTable = obj.qTable;
            Serialnumber = obj.serial_number;

            num_joints=obj.joint_num;

            % 左侧端点从qTable中找
            x0=qTable.q(:,Serialnumber);
            vx0=qTable.vq(:,Serialnumber);
            ax0=qTable.aq(:,Serialnumber);

            % 右侧端点
            x1=qTable.q(:,Serialnumber+1);
            vx1=parameters(1:num_joints)';
            ax1=parameters(num_joints+1:num_joints*2)';
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
            %{
            a00=x0;
            a01=vx0;
            a02=ax0/2;
            a03=(4*x1-vx1*t1-4*x0-3*vx0*t1-ax0*t1^2)/t1^3;
            a04=(vx1*t1-3*x1+3*x0+2*vx0*t1+ax0*t1^2/2)/t1^4;
            ax1=2*a02+6*a03*t1+12*a04*t1^2;
            %}
            a00=x0;
            a01=vx0;
            a02=ax0/2;
            a03=(20*x1-20*x0-(8*vx1+12*vx0)*t1-(3*ax0-ax1)*t1^2)/(2*t1^3);
            a04=(30*x0-30*x1+(14*vx1+16*vx0)*t1+(3*ax0-2*ax1)*t1^2)/(2*t1^4);
            a05=(12*x1-12*x0-(6*vx1+6*vx0)*t1-(ax0-ax1)*t1^2)/(2*t1^5);
            A1 = [a00, a01, a02, a03, a04, a05];

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
            result=[A1*[tl1.^0; tl1.^1; tl1.^2; tl1.^3; tl1.^4; tl1.^5],...
                    A2*[tl2.^0; tl2.^1; tl2.^2; tl2.^3; tl2.^4; tl2.^5]; % the angle varies of each joint
                    A1*[z1; tl1.^0; 2*tl1.^1; 3*tl1.^2; 4*tl1.^3; 5*tl1.^4],...
                    A2*[z2; tl2.^0; 2*tl2.^1; 3*tl2.^2; 4*tl2.^3; 5*tl2.^4]; % the velocity varies of each joint
                    A1*[z1; z1; 2*tl1.^0; 6*tl1.^1; 12*tl1.^2; 20*tl1.^3],...
                    A2*[z2; z2; 2*tl2.^0; 6*tl2.^1; 12*tl2.^2; 20*tl2.^3]; % the acceleration varies of each joint
                   ];
            if Serialnumber+2>size(qTable.q,2) %处理末端
                result=result(:,1:spacenum+1);
            end
        end
    end
end

