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
        hyperparameter; %外部可调参数
    end
    
    methods
        function obj = fitnessFun_ap1(manipulator_model)
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
            thetas=[obj.previousJPos,obj.previousJPos+parameters'];
            %{
              fq
            %}
            fq=norm(parameters);
            [cost,cost_vec] = obj.evaluatePosture(thetas(:,2));
            cost_vec=[cost_vec,fq];
            cost=[cost, fq]*[1,obj.hyperparameter.ap1_to2]';
            fitness_value=1/(cost+eps); %防止/0错误
        end
        function [cost,cost_vec] = evaluatePosture(obj,theta)
            trans=fastForwardTrans(obj,theta); %forwardTrans to get the transform matrix
            %{
              oa表示避障指标
            %}
            %collision_count=0; %用于统计轨迹上机械臂与障碍物的碰撞次数
            oa=0;
            if obj.hyperparameter.ap1_obflag==true
            min_dis=1;
            for j=1:obj.joint_num
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
            if min_dis>obj.hyperparameter.ob_e
                oa=1/min_dis;
            else% min_dis<=ob_e
                oa=1/obj.hyperparameter.ob_e;
            end
            oa=(oa*obj.hyperparameter.ob_e)^obj.hyperparameter.ob_beta;
            end
            %{
              fdt表示机械臂末端与给定路径点的相符程度度量（越小越好）
            %}
            pos=trans(1:3,4,end);
            fdt=norm(obj.target_pos-pos);
            cost_vec=[fdt,oa];
            cost=cost_vec*[1,obj.hyperparameter.ap1_to1]';
        end
        function T = fastForwardTrans(obj, theta)
            a=obj.a; d=obj.d; alpha=obj.alpha; 
            base=obj.base;
            assert(size(theta,1)==obj.joint_num)
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
    end
end

