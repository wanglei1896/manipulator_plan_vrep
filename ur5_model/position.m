function [x,y,z]=position(theta)
    %已知关节角求变换矩阵
% UR5 
    a=[0,-0.42500,-0.39225,0,0,0];
    d=[0.089159,0,0,0.10915,0.09465,0.08230];
    alpha=[pi/2,0,0,pi/2,-pi/2,0];
% LBR-iiwa_7_R800 D-H
%     a=[0 1.2407-1.0612 0 1.6405-1.4612 0 1.9469-1.8612 0];
%     d=[1.0612-0.87375 0 1.4612-1.2407 0 1.8612-1.6405 0 1.9722-1.9469];
%     alpha=[-pi/2,-pi/2,pi/2,pi/2,-pi/2,-pi/2,0];
    
    T01=T_para(theta(1),d(1),a(1),alpha(1));
    T12=T_para(theta(2),d(2),a(2),alpha(2));
    T23=T_para(theta(3),d(3),a(3),alpha(3));
    T34=T_para(theta(4),d(4),a(4),alpha(4));
    T45=T_para(theta(5),d(5),a(5),alpha(5));
    T56=T_para(theta(6),d(6),a(6),alpha(6));
    %T67=T_para(theta(7),d(7),a(7),alpha(7));
    
    T02=T01*T12;
    T03=T02*T23;
    T04=T03*T34;
    T05=T04*T45;
    T06=T05*T56;
    %T07=T06*T67;

	x=[0;T01(1,4);T02(1,4);T03(1,4);T04(1,4);T05(1,4);T06(1,4)];%T07(1,4)];
	y=[0;T01(2,4);T02(2,4);T03(2,4);T04(2,4);T05(2,4);T06(2,4)];%T07(2,4)];
	z=[0;T01(3,4);T02(3,4);T03(3,4);T04(3,4);T05(3,4);T06(3,4)];%T07(3,4)];