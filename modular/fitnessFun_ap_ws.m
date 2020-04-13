classdef fitnessFun_ap_ws
    %FITNESSFUN_WORKSPACE
    %   在末端任务空间进行路径优化的适应度函数，检验比较两路径相似程度的指标
    
     properties
        % 访问全局变量太慢，所以存在这里作为私有数据
        spacenum; % 生成轨迹段数
        qTable; % 各轨迹段端点处的参数值（关节位置、速度、加速度）
        serial_number; %目前优化的是第几段
        target_path; %目前要优化段的目标路径
        parameter_bound;
        nd; %维度大小
    end
    
    methods
        
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
            ql=result(1:obj.nd,:);
            vl=result(obj.nd+1:obj.nd*2,:);
            %{
              ft表示轨迹中速度/加速度超过max的轨迹片段的各采样点速度/加速度之和。
              此指标可发现轨迹中速度/加速度过高的片段，并引导agent减少或改善这些片段；
              此指标对低于max的轨迹片段无指导，适合作为惩罚项；
              此指标易引发运动时间(t1、t2)的扩张，最好配合'time'指标使用
            %}
            ft=0;
            maxv=1/4;
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
            fq=0;
            %{
              fdt表示机械臂末端划过的路径与给定路径的相符程度度量（越小越好）
            %}
            regPos = regular_path(ql,size(obj.target_path,2)-1);
            deltas=abs(obj.target_path-regPos);
            Pos_punishment=[];
            fdt=1;
            for delta=deltas(:,2:end)
                if sum(delta)>0.001
                    Pos_punishment=[Pos_punishment,sum(delta)];
                else
                    Pos_punishment=[Pos_punishment,0];
                end
                fdt=fdt*(sum(delta)+1);
            end
            fdt=fdt*(sum(deltas(:,end))+1);
            fdt=log10(fdt);
            %fdt=sum(sum(deltas));
            %{
              fdis表示机械臂末端划过的轨迹长度
            %}
            dx=diff(ql(1,:)); dy=diff(ql(2,:)); dz=diff(ql(3,:));
            dis=sqrt(dx.^2+dy.^2+dz.^2);
            fdis=sum(dis);
            %{
              time表示轨迹运动的时间
            %}
            time=sum(parameters(end));
            cost_vec=[ft,fq,fdt,fdis,time, Pos_punishment];
            cost=cost_vec*[0,0,1,0,0, zeros(1,length(Pos_punishment))]';
            evaluate_value=1/(cost+eps); %防止/0错误
        end
        
        function [status, result] = convertSolutionToTrajectory(obj, parameters)
            status = 0;
            spacenum = obj.spacenum;
            qTable = obj.qTable;
            Serialnumber = obj.serial_number;
            nd =obj.nd;

            % 左侧端点从qTable中找
            x0=qTable.q(:,Serialnumber);
            vx0=qTable.vq(:,Serialnumber);
            ax0=qTable.aq(:,Serialnumber);

            % 右侧端点来自优化向量
            x1=parameters(1:nd)';
            vx1=parameters(nd+1:nd*2)';
            ax1=parameters(nd*2+1:nd*3)';
            t1=parameters(end);
            assert(t1>0);

            assert(isequal(size(x0),[nd,1])); % 需为列向量
            assert(isequal(size(x1),[nd,1])); % 需为列向量
            assert(isequal(size(vx1),[nd,1])); % 需为列向量

            % compute interpolate factor(analytical solution)
            %{
            a00=x0;
            a01=vx0;
            a02=ax0/2;
            a03=(4*x1-vx1*t1-4*x0-3*vx0*t1-ax0*t1^2)/t1^3;
            a04=(vx1*t1-3*x1+3*x0+2*vx0*t1+ax0*t1^2/2)/t1^4;
            a05=t1^5;
            A1 = [a00, a01, a02, a03, a04, a05];
            %}
            A=[1 0 0 0 0 0;
               0 1 0 0 0 0;
               0 0 2 0 0 0;
               1 t1 t1^2 t1^3 t1^4 t1^5;
               0 1 2*t1 3*t1^2 4*t1^3 5*t1^4;
               0 0 2 6*t1 12*t1^2 20*t1^3];
            b=[x0'; vx0'; ax0'; x1'; vx1'; ax1'];
            A1 = A\b;

            tl1=linspace(0,t1,spacenum+1);
            
            z1=zeros(1,length(tl1));
            result=[A1'*[tl1.^0; tl1.^1; tl1.^2; tl1.^3; tl1.^4; tl1.^5]; % the angle varies of each joint
                    A1'*[z1; tl1.^0; 2*tl1.^1; 3*tl1.^2; 4*tl1.^3; 5*tl1.^4]; % the velocity varies of each joint
                    A1'*[z1; z1; 2*tl1.^0; 6*tl1.^1; 12*tl1.^2; 20*tl1.^3]; % the acceleration varies of each joint
                   ];
        end
    end
end

