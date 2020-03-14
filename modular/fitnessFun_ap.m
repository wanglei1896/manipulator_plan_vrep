classdef fitnessFun_ap
    %FITNESSFUN_AP �˴���ʾ�йش����ժҪ
    %   ��·���˶�(along path)�滮�Ĵ��ۺ���
    
    properties
        % ����ȫ�ֱ���̫�������Դ���������Ϊ˽������
        d; a; alpha; joint_num; % ��е�۵���ز���
        spacenum; % ���ɹ켣����
        qTable; % ���켣�ζ˵㴦�Ĳ���ֵ���ؽ�λ�á��ٶȡ����ٶȣ�
        serial_number; %Ŀǰ�Ż����ǵڼ���
        target_path; %ĿǰҪ�Ż��ε�Ŀ��·��
        parameter_bound;
    end
    
    methods
        function obj = fitnessFun_ap(manipulator_model)
            obj.d = manipulator_model.d;
            obj.a = manipulator_model.a;
            obj.alpha = manipulator_model.alpha;
        end
        
        function fitness_value = fitnessf(obj, parameters)
            % ���Ż��㷨�ص��ã�ע��ӿ����䱣��һ��
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
                  ft��ʾ�켣���ٶ�/���ٶȳ���max�Ĺ켣Ƭ�εĸ��������ٶ�/���ٶ�֮�͡�
                  ��ָ��ɷ��ֹ켣���ٶ�/���ٶȹ��ߵ�Ƭ�Σ�������agent���ٻ������ЩƬ�Σ�
                  ��ָ��Ե���max�Ĺ켣Ƭ����ָ�����ʺ���Ϊ�ͷ��
                  ��ָ���������˶�ʱ��(t1��t2)�����ţ�������'time'ָ��ʹ��
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
                  fdis��ʾ��е��ĩ�˻�����·����Ҫ��·��������̶ȶ�����ԽСԽ�ã�
                %}
                pos=zeros(3,size(ql,2));
                for i=1:size(ql,2)
                    pos(:,i) = fastForwardTrans(6, ql(:,i));
                end
                regPos = regular_path(pos,size(obj.target_path,2)-1);
                fdis=sum(sum(abs(obj.target_path-regPos)));
                %{
                  time��ʾ�켣�˶���ʱ��
                %}
                time=sum(parameters(end));
                cost_vec=[ft,fq,fdis,time];
                cost=cost_vec*[0,0,1,0]';
                evaluate_value=1/(cost+eps); %��ֹ/0����
            end
            function pos = fastForwardTrans(number, theta)
                a=obj.a; d=obj.d; alpha=obj.alpha;
                % toolbox���Դ������˶�ѧҪ���ö���̫���������Ż�һ������İ汾
                % number��ʾ�ؽڱ�ţ�0-5�Źؽڣ�6��ĩ��
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
            status = 0;
            spacenum = obj.spacenum;
            qTable = obj.qTable;
            Serialnumber = obj.serial_number;

            q_final = parameters(1:6);
            vq_final = parameters(7:12);
            time = parameters(13);
            assert(time>0);

            num_joints=length(q_final);

            % decode configuration vector
            % to get initial and final point joint configuration
            x0=qTable.q(Serialnumber);
            vx0=qTable.vq(Serialnumber);
            ax0=qTable.aq(Serialnumber);

            % to get middle point joint-configuration and time-interval
            x1=q_final';
            vx1=vq_final';
            t1=time;

            assert(isequal(size(x0),[num_joints,1])); % ��Ϊ������
            assert(isequal(size(x1),[num_joints,1])); % ��Ϊ������
            assert(isequal(size(vx1),[num_joints,1])); % ��Ϊ������

            % compute interpolate factor(analytical solution)
            a00=x0;
            a01=vx0;
            a02=ax0/2;
            a03=(4*x1-vx1*t1-4*x0-3*vx0*t1-ax0*t1^2)/t1^3;
            a04=(vx1*t1-3*x1+3*x0+2*vx0*t1+ax0*t1^2/2)/t1^4;
            A1 = [a00, a01, a02, a03, a04];
%{
            tl=linspace(0,t1+t2,spacenum+1);
            middlespace=floor(spacenum*t1/(t1+t2))+1; %[1,spacenum]
            % disp([middlespace t1 t2])
            tl1=tl(1:middlespace); % at least 1:1
            tl2=tl(middlespace+1:end)... % at least end:end
                   -time1; % ���ڼ���A2��Ĭ�Ϲ켣��0ʱ�̿�ʼ
%}
            tl1=linspace(0,t1,spacenum+1);
            
            z1=zeros(1,length(tl1));
            result=[A1*[tl1.^0; tl1.^1; tl1.^2; tl1.^3; tl1.^4]; % the angle varies of each joint
                    A1*[z1; tl1.^0; 2*tl1.^1; 3*tl1.^2; 4*tl1.^3]; % the velocity varies of each joint
                    A1*[z1; z1; 2*tl1.^0; 6*tl1.^1; 12*tl1.^2]; % the acceleration varies of each joint
                   ];
        end
    end
end

