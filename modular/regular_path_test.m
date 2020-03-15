function regular_path_test()
    spacenum=20;
    path1=[0,1;
           0,1;
           0,1];
    rr=[linspace(0,1,spacenum+1);linspace(0,1,spacenum+1);linspace(0,1,spacenum+1)];
    r=regular_path(path1, spacenum);
    %disp(r)
    %disp(rr)
    assert(sum(sum(abs(rr-r)))<0.00001);
    
    path2=[0,1,1;
           0,0,0;
           0,0,1];
    rr=[[linspace(0,1,11), ones(1,10)];
        zeros(1,21);
        [zeros(1,10), linspace(0,1,11)]];
    r=regular_path(path2, spacenum);
    assert(sum(sum(abs(rr-r)))<0.00001);
    
    spacenum=5;
    rr=[0,0.4,0.8,  1,  1,1;
        0,  0,  0,  0,  0,0;
        0,  0,  0,0.2,0.6,1];
    r=regular_path(path2, spacenum);
    assert(sum(sum(abs(rr-r)))<0.00001);
    
    spacenum=3;
    path3=[0,1,1;
           0,1,2;
           0,0,0];
    rr=[0,(1+sqrt(1/2))/3,              1,1;
        0,(1+sqrt(1/2))/3,2-(sqrt(2)+1)/3,2;
        0,              0,              0,0];
    r=regular_path(path3, spacenum);
    assert(sum(sum(abs(rr-r)))<0.00001);
end

