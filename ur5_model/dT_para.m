function T = dT_para(theta,d,a,alpha)
    T=[-sin(theta),-cos(theta)*cos(alpha),cos(theta)*sin(alpha),-a*sin(theta);
        cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta);
        0,0,0,0;
        0,0,0,0];
end