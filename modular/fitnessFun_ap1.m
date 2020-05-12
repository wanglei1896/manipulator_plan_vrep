classdef fitnessFun_ap1
    %FITNESSFUN_AP 此处显示有关此类的摘要
    %   沿路径运动(along path)规划的代价函数1
    %   寻找满足条件的单个关节位姿
    
    properties
        d; a; alpha; base; joint_num; offset; % 机械臂的相关参数
        linkShapes; obstacles; % 机械臂连杆和障碍物的mesh shape（顶点表示）
        linkCentre; obsCentre;
        target_pos; %目前要优化的目标位置
        previousJPos; %上一个关节角度
        parameter_bound;
        jointPath; %规划出的关节空间对应路径
    end
    
    methods
        function obj = fitnessFun_ap1(manipulator_model)
            obj.d = manipulator_model.d;
            obj.a = manipulator_model.a;
            obj.alpha = manipulator_model.alpha;
            obj.offset = manipulator_model.offset;
            obj.base = manipulator_model.base.t;
        end
        
        function [fitness_value,cost_vec] = fitnessf(obj, parameters)
            global hyperparameter
            % 给优化算法回调用，注意接口与其保持一致
            thetas=[obj.previousJPos,obj.previousJPos+parameters'];
            %{
              fq
            %}
            fq=norm(parameters);
            [cost,cost_vec] = obj.evaluatePosture(thetas(:,2));
            cost_vec=[cost_vec,fq];
            cost=[cost, fq]*[1,hyperparameter.ap1_to1]';
            fitness_value=1/(cost+eps); %防止/0错误
        end
        function [cost,cost_vec] = evaluatePosture(obj,theta)
            global hyperparameter
            %{
              oa表示避障指标
            %}
            %collision_count=0; %用于统计轨迹上机械臂与障碍物的碰撞次数
            min_dis=1;
            trans=fastForwardTrans(obj,theta); %forwardTrans to get the transform matrix
            for j=1:6
                tran = trans(:,:,j+1);
                for k=1:length(obj.obstacles)
                    vertices = tran(1:3,1:3)*obj.linkShapes(j).vex+tran(1:3,4);
                    % Do collision detection
                    dis=openGJK(vertices,obj.obstacles(k).vex);
                    if dis<min_dis
                        %collision_count=collision_count+1;
                        min_dis=dis;
                    end
                end
            end 
            %if min_dis>=1e-2
            %    oa=0;
            %else
            if min_dis>hyperparameter.ob_e
                oa=1/min_dis;
            else% min_dis<=ob_e
                oa=1/hyperparameter.ob_e;
            end
            oa=(oa*hyperparameter.ob_e)^hyperparameter.ob_beta;
            %{
              fdt表示机械臂末端与给定路径点的相符程度度量（越小越好）
            %}
            pos=trans(1:3,4,end);
            fdt=norm(obj.target_pos-pos);
            cost_vec=[fdt,oa];
            cost=cost_vec*[1,hyperparameter.ap1_to2]';
        end
        function T = fastForwardTrans(obj, theta)
            a=obj.a; d=obj.d; alpha=obj.alpha; 
            base=eye(4); base(1:3,4)=obj.base;
            assert(size(theta,1)==6)
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
    end
end

