classdef fitnessFun_p2p
    %FITNESSFUN_P2P 此处显示有关此类的摘要
    %   点到点规划的代价函数
    
    properties
        % 访问全局变量太慢，所以存在这里作为私有数据
        d; a; alpha; joint_num; % 机械臂的相关参数
        spacenum; % 生成轨迹段数
        qStart; qFinal;
        
        parameter_bound;
    end
    
    methods
        function obj = fitnessFun_p2p(manipulator_model)
            obj.d = manipulator_model.d;
            obj.a = manipulator_model.a;
            obj.alpha = manipulator_model.alpha;
        end
        
        function fitness_value = fitnessf(obj, parameters)
            % 给优化算法回调用，注意接口与其保持一致
            [status, result] = obj.convertSolutionToTrajectory(parameters);
            if status ~= 0
                fitness_value = 1/(result*1000);
                return;
            end
            fitness_value = evaluateTrajectory(result);

            function evaluate_value = evaluateTrajectory(result)
                ql=result(1:6,:);
                vl=result(7:12,:);
                al=result(13:18,:);
                %{
                  ft表示轨迹中速度/加速度超过max的轨迹片段的各采样点速度/加速度之和。
                  此指标可发现轨迹中速度/加速度过高的片段，并引导agent减少或改善这些片段；
                  此指标对低于max的轨迹片段无指导，适合作为惩罚项；
                  此指标易引发运动时间(t1、t2)的扩张，最好配合'time'指标使用
                %}
                ft=0;
                maxv=pi/4;
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
                  fdis表示机械臂末端划过的轨迹长度
                %}
                pos=zeros(3,size(ql,2));
                for i=1:size(ql,2)
                    %pos(:,i) = manipulator.A(6, ql(:,i)).t;
                    pos(:,i) = fastForwardTrans(6, ql(:,i));
                end
                dx=diff(pos(1,:)); dy=diff(pos(2,:)); dz=diff(pos(3,:));
                dis=sqrt(dx.^2+dy.^2+dz.^2);
                fdis=sum(dis);
                %{
                  time表示轨迹运动的时间
                %}
                time=sum(parameters(end-1:end));
                cost_vec=[ft,fq,fdis,time];
                cost=cost_vec*[0,0,1,0]';
                evaluate_value=1/(cost+eps); %防止/0错误
            end
            function pos = fastForwardTrans(number, theta)
                a=obj.a; d=obj.d; alpha=obj.alpha;
                % toolbox中自带的正运动学要调用对象，太慢，这里优化一个更快的版本
                % number表示关节编号，0-5号关节，6号末端
                if number == 0
                    pos = [0 0 0]'; return;
                end
                T01=T_para(theta(1),d(1),a(1),alpha(1));
                if number == 1
                    pos = T01(1:3,4); return;
                end
                T12=T_para(theta(2),d(2),a(2),alpha(2));
                T02=T01*T12;
                if number == 2
                    pos = T02(1:3,4); return;
                end
                T23=T_para(theta(3),d(3),a(3),alpha(3));
                T03=T02*T23;
                if number == 3
                    pos = T03(1:3,4); return;
                end
                T34=T_para(theta(4),d(4),a(4),alpha(4));
                T04=T03*T34;
                if number == 4
                    pos = T04(1:3,4); return;
                end
                T45=T_para(theta(5),d(5),a(5),alpha(5));
                T05=T04*T45;
                if number == 5
                    pos = T05(1:3,4); return;
                end
                T56=T_para(theta(6),d(6),a(6),alpha(6));
                T06=T05*T56;
                if number == 6
                    pos = T06(1:3,4); return;
                end
                %positions=[[0 0 0]', T01(1:3,4), T02(1:3,4), T03(1:3,4), T04(1:3,4), T05(1:3,4), T06(1:3,4)];
                %pos = positions(:,number+1);

                function T = T_para(theta,d,a,alpha)
                    T=[cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
                       sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
                       0,sin(alpha),cos(alpha),d;
                       0,0,0,1];
                end
            end
        end
        
        function [status, result] = convertSolutionToTrajectory(obj, parameters)
            status=0;
            spacenum = obj.spacenum;
            qStart = obj.qStart;
            qFinal = obj.qFinal;

            q_middle = parameters(1:6);
            vq_middle = parameters(7:12);
            time1 = parameters(13);
            time2 = parameters(14);
            assert(time1>0 && time2>0);

            num_joints=length(q_middle);

            % decode configuration vector
            % to get initial and final point joint configuration
            x0=qStart';
            vx0=zeros(num_joints,1);
            ax0=zeros(num_joints,1);
            x2=qFinal';
            vx2=zeros(num_joints,1);
            ax2=zeros(num_joints,1);

            % to get middle point joint-configuration and time-interval
            x1=q_middle';
            vx1=vq_middle';
            t1=time1;
            t2=time2;

            assert(isequal(size(x0),[num_joints,1])); % 需为列向量
            assert(isequal(size(x2),[num_joints,1])); % 需为列向量
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
%{
            tl=linspace(0,t1+t2,spacenum+1);
            middlespace=floor(spacenum*t1/(t1+t2))+1; %[1,spacenum]
            % disp([middlespace t1 t2])
            tl1=tl(1:middlespace); % at least 1:1
            tl2=tl(middlespace+1:end)... % at least end:end
                   -time1; % 由于计算A2是默认轨迹从0时刻开始
%}
            spacenum1=floor(spacenum/2);
            spacenum2=spacenum-spacenum1;
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
        end
    end
end

