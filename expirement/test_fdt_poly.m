% 测试用多项式拟合路径时fdt指标的性质
function test_fdt_poly()
global tfp
    spacenum = 10;
    target_path = linspace(0,0,11);
    x0=0; vx0=0; ax0=0;
    result=zeros(11,101,101);
    traj_his=zeros(11,101,101);
    i=0; 
    for x=linspace(-1,1,101)
        i=i+1;
        j=0;
        for v=linspace(-10,10,101)
            j=j+1;
            traj=convert([x,v]);
            traj_his(:,j,i)=traj(1,:);
            result(:,j,i)=evaluate(traj(1,:));
        end
    end
    fdt_result=sum(result,1);
    tfp.result=result;
    tfp.fdt_result=fdt_result(1);
    tfp.traj_his=traj_his;
    
    function Pos_punishment = evaluate(ql)
        regPos = regular_path(ql,size(target_path,2)-1);
        deltas=abs(target_path-regPos);
        Pos_punishment=[];
        for delta=deltas
            Pos_punishment=[Pos_punishment,sum(delta)];
        end
    end
    function result = convert(parameters)
        % 右侧端点来自优化向量
        x1=parameters(1);
        vx1=parameters(2);
        t1=1;

        % compute interpolate factor(analytical solution)
        a00=x0;
        a01=vx0;
        a02=ax0/2;
        a03=(4*x1-vx1*t1-4*x0-3*vx0*t1-ax0*t1^2)/t1^3;
        a04=(vx1*t1-3*x1+3*x0+2*vx0*t1+ax0*t1^2/2)/t1^4;
        A1 = [a00, a01, a02, a03, a04];

        tl1=linspace(0,t1,spacenum+1);

        z1=zeros(1,length(tl1));
        result=[A1*[tl1.^0; tl1.^1; tl1.^2; tl1.^3; tl1.^4]; % the angle varies of each joint
                A1*[z1; tl1.^0; 2*tl1.^1; 3*tl1.^2; 4*tl1.^3]; % the velocity varies of each joint
                A1*[z1; z1; 2*tl1.^0; 6*tl1.^1; 12*tl1.^2]; % the acceleration varies of each joint
               ];
    end
end

