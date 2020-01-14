function [Jacob0]=Jacob(theta)
    %已知关节角求变换矩阵
    
    a=[0,-0.42500,-0.39225,0,0,0];
    d=[0.089159,0,0,0.10915,0.09465,0.08230];
    alpha=[pi/2,0,0,pi/2,-pi/2,0];

    
    T01=T_para(theta(1),d(1),a(1),alpha(1));
    T12=T_para(theta(2),d(2),a(2),alpha(2));
    T23=T_para(theta(3),d(3),a(3),alpha(3));
    T34=T_para(theta(4),d(4),a(4),alpha(4));
    T45=T_para(theta(5),d(5),a(5),alpha(5));
    T56=T_para(theta(6),d(6),a(6),alpha(6));
    
    dT01=dT_para(theta(1),d(1),a(1),alpha(1));
    dT12=dT_para(theta(2),d(2),a(2),alpha(2));
    dT23=dT_para(theta(3),d(3),a(3),alpha(3));
    dT34=dT_para(theta(4),d(4),a(4),alpha(4));
    dT45=dT_para(theta(5),d(5),a(5),alpha(5));
    dT56=dT_para(theta(6),d(6),a(6),alpha(6));
    
    Je1=dT01*T12*T23*T34*T45*T56*[0;0;0;1];
    Je2=T01*dT12*T23*T34*T45*T56*[0;0;0;1];
    Je3=T01*T12*dT23*T34*T45*T56*[0;0;0;1];
    Je4=T01*T12*T23*dT34*T45*T56*[0;0;0;1];
    Je5=T01*T12*T23*T34*dT45*T56*[0;0;0;1];
    Je6=T01*T12*T23*T34*T45*dT56*[0;0;0;1];
    
    
    Jacob0(:,1)=Je1(1:3,1);
    Jacob0(:,2)=Je2(1:3,1);
    Jacob0(:,3)=Je3(1:3,1);
    Jacob0(:,4)=Je4(1:3,1);
    Jacob0(:,5)=Je5(1:3,1);
    Jacob0(:,6)=Je6(1:3,1);