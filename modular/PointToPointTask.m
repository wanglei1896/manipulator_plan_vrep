classdef PointToPointTask < TaskEnvironment
    %POINTTOPOINTTASK 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        initial_position=[1 0]'
        final_position=[1 2]'
        solution_bound
        prefer  %要优化的指标
        obstacles
    end
    
    methods
        function obj = PointToPointTask(algorithm, manipulator)
            obj@TaskEnvironment(algorithm,manipulator);
            njoint=length(manipulator.joint_angle);
            obj.updateSoulutionBound(njoint);
            obj.obstacles=[];
        end
        function weight_vec=weight_vec(obj)
            weight_vec=[ 2;  %ft 2
                       1.8;  %fq 1.8
                         2;  %fdis 2
                         1]; %time 1
            %ft:力矩的惩罚项，不超过最大值时为0,这里用加速度代替
            %fq:各关节移动总幅度
            %fdis:轨迹总长
            %time:经过时间
            if strcmp(obj.prefer,'time')
                weight_vec = [2;0;0;1];
            elseif strcmp(obj.prefer,'fq')
                weight_vec = [2;1;0;0];
            elseif strcmp(obj.prefer,'fdis')
                weight_vec = [2;0;1;0];
            end
        end
        function solution_dimension = solution_dimension(obj)
            solution_dimension = length(obj.solution_bound);
        end
        function updateSoulutionBound(obj, njoint)
            % solution_bound结构数组中(1)代表下限，(2)代表上限
            solution_bound_s(1).qm = -pi*ones(njoint,1);  %中间位置关节角度
            solution_bound_s(2).qm = pi*ones(njoint,1);
            solution_bound_s(1).vqm = -pi/4*ones(njoint, 1);  %中间位置关节速度
            solution_bound_s(2).vqm = pi/4*ones(njoint, 1);
            solution_bound_s(1).phis = -pi*ones(njoint-2, 1);  %起始位置冗余参数
            solution_bound_s(2).phis = pi*ones(njoint-2, 1);
            solution_bound_s(1).phif = -pi*ones(njoint-2, 1);  %终点位置冗余参数
            solution_bound_s(2).phif = pi*ones(njoint-2, 1);
            solution_bound_s(1).t1 = 0.1;  %第一段时间
            solution_bound_s(2).t1 = 8;
            solution_bound_s(1).t2 = 0.1;  %第二段时间
            solution_bound_s(2).t2 = 8;
            obj.solution_bound = obj.SolutionStruct2Vector(solution_bound_s);
        end
        function result = SolutionStruct2Vector(obj, sol_struct)
            result = [];
            result = [result; [sol_struct.qm]];
            result = [result; [sol_struct.vqm]];
            result = [result; [sol_struct.phis]];
            result = [result; [sol_struct.phif]];
            result = [result; [sol_struct.t1]];
            result = [result; [sol_struct.t2]];
        end
        function result = SolutionVector2Struct(obj, sol_vec)
            njoint=length(obj.intaskManipulator.joint_angle);
            if njoint<=1
                error("can't less than 1 joint");
            end
            result.qm = sol_vec(1:njoint);
            result.vqm = sol_vec(njoint+1:2*njoint);
            result.phis = sol_vec(2*njoint+1:3*njoint-2);
            result.phif = sol_vec(3*njoint-1:4*njoint-4);
            result.t1 = sol_vec(end-1);
            result.t2 = sol_vec(end);
        end
        function resetIntaskManipulator(obj,manipulator)
            obj.intaskManipulator = manipulator;
            njoint=length(manipulator.joint_angle);
            obj.updateSoulutionBound(njoint);
        end
        function addObstacle(obj, obstacle)
            if ~isa(obstacle, 'Obstacle')
                error('should be Obstacle');
            end
            obj.obstacles = [obj.obstacles, obstacle];
        end
        function clearObstacle(obj)
            obj.obstacles = [];
        end
        
        function initialEnvironment(obj,config_initial_c,config_final_c)
            if nargin>1
                obj.initial_position = config_initial_c;
                obj.final_position = config_final_c;
            end
            % 设定点不能在机械臂的工作范围外
            if ~(obj.intaskManipulator.isReachin(obj.initial_position)...
                    &&obj.intaskManipulator.isReachin(obj.final_position))
                warndlg('there is no redundancy..... change start position','!! error !!')
                clear
                return
            end
        end
        
        function fitness_value = fitnessfun(obj, parameters)
            %状态码为零表示转化成功
            [status, result] = obj.convertSolutionToTrajectory(parameters);
            if status ~= 0
                fitness_value = 1/(result*1000);
                return;
            end
            fitness_value = obj.evaluateTrajectory(result);  
        end
        function evaluate_value = evaluateTrajectory(obj, trajectory)
            ft=obj.intaskManipulator.sum_torque(trajectory);
            angle_sets = trajectory.toPointSet;
            fq=obj.intaskManipulator.odometer(angle_sets);
            fdis=obj.intaskManipulator.cartesian_odometer(angle_sets);
            time=trajectory.duration_time;
            fitness_vec=[ft,fq,fdis,time];
            cost=fitness_vec*obj.weight_vec;
            evaluate_value=1/cost;
            if ~isempty(obj.obstacles)
                tp=[];
                for t=linspace(trajectory.initial_time, trajectory.final_time, 10)
                    obj.intaskManipulator.joint_angle=trajectory.getPositionAt(t);
                    tp=[tp, obj.intaskManipulator.forwardTrans];
                    if obj.obstacles.isLappedWithEntity(obj.intaskManipulator)
                        evaluate_value=0;
                        return;
                    end
                end
                point_sets=[obj.intaskManipulator.toSurfacePointSet(angle_sets(:,1),2),tp,...
                    fliplr(obj.intaskManipulator.toSurfacePointSet(angle_sets(:,end),2))];
                obstacle_point_set=obj.obstacles.toSurfacePointSet;
                in=inpolygon(obstacle_point_set(1,:),obstacle_point_set(2,:),...
                        point_sets(1,:),point_sets(2,:));
                if any(any(in))
                    evaluate_value=0;
                end
            end
        end
        function drawEnvironment(obj)
            % 绘制障碍物
            if ~isempty(obj.obstacles)
                obj.obstacles.draw();
                hold on
            end
            canvas_size=norm(obj.intaskManipulator.link_length,1)+1;
            % 绘制机械臂
            obj.intaskManipulator.draw();
            % 绘制起点与终点
            hold on
            axis([-1 1 -1 1]*canvas_size)
            plot(obj.initial_position(1),obj.initial_position(2),'gx')
            plot(obj.final_position(1),obj.final_position(2),'rx')
            xlabel('x (m)')
            ylabel('y (m)')
            hold off
        end
        function handle = simulate(obj,trajectory)
            if nargin==1
                if ~isa(obj.trained_trajectory, 'Trajectory')
                    error("train failed, can't simulate");
                end
                trajectory = obj.trained_trajectory;
            end
            
            tc = [];
            for time=linspace(0,trajectory.final_time,50)
                joint_position = trajectory.getPositionAt(time);
                obj.intaskManipulator.joint_angle = joint_position;
                obj.drawEnvironment();
                hold on
                % 画轨迹
                tc = [tc, obj.intaskManipulator.forwardTrans];
                plot(tc(1,:),tc(2,:))
                hold off
                pause(0)
            end
            figure,
            obj.intaskManipulator.joint_angle = trajectory.getPositionAt(trajectory.middle_time);
            obj.drawEnvironment();
            hold on
            obj.intaskManipulator.joint_angle = trajectory.getPositionAt(trajectory.initial_time);
            obj.drawEnvironment();
            hold on
            obj.intaskManipulator.joint_angle = trajectory.getPositionAt(trajectory.final_time);
            obj.drawEnvironment();
            hold on
            tc=[tc,obj.intaskManipulator.forwardTrans];
            
            ta(1)=annotation('textarrow',[.3,.4],[.3,.4]);ta(2)=annotation('textarrow',[.3,.4],[.2,.3]);
            ta(3)=annotation('textarrow',[.3,.4],[.1,.2]);
            ta(1).String='initial position';ta(1).FontSize=16;
            ta(2).String='middle position';ta(2).FontSize=16;
            ta(3).String='final position';ta(3).FontSize=16;
            handle = plot(tc(1,:),tc(2,:),'b-');
            hold off
        end
        function handle = plotCostHistory(obj)
            handle = plot(1:length(obj.fitness_history),obj.fitness_history,'b-');
            if max(obj.fitness_history)>obj.fitness_history(end)*12
                ylim([0 obj.fitness_history(end)*12]);
            end
            xlabel iteration;
        end
    end
    
    methods %(Access=protected)
        function [status,result]=convertSolutionToTrajectory(obj,parameters)
            solution = obj.SolutionVector2Struct(parameters');
            %generate trajctory in joint space by linear interplot
            [status_sign,k] = obj.intaskManipulator.isPosturePossible(...
                                    [obj.initial_position, obj.final_position], [solution.phis, solution.phif]);
            status = sum(~status_sign);
            if(status > 0)
                result = sum(k.*(k>1));
                return;
            end
           	q_initial_position = obj.intaskManipulator.inverseTrans(obj.initial_position, solution.phis);
         	q_final_position = obj.intaskManipulator.inverseTrans(obj.final_position, solution.phif);
            num_joints=length(q_initial_position);

            % decode configuration vector
            % to get initial and final point joint configuration
            x0=q_initial_position;
            vx0=zeros(num_joints,1);
            ax0=zeros(num_joints,1);
            x2=q_final_position;
            vx2=zeros(num_joints,1);
            ax2=zeros(num_joints,1);

            % to get middle point joint-configuration and time-interval
            x1=solution.qm;
            vx1=solution.vqm;
            t1=solution.t1;
            t2=solution.t2;
            if t2==0
                t2=0.1;
            end

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
            
            trajectory1 = PolynomialTrajectory(A1',0,t1);
            trajectory2 = PolynomialTrajectory(A2',0,t2);
            result = TPBSO_Trajectory(trajectory1, trajectory2);
        end
    end
end

