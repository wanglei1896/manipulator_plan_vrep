function regular_JointPos_test()
    qs1=0; qf1=0; rqs1=0; rqf1=0;
    [s,f]=regular_JointPos(qs1,qf1);
    assert(isequal([rqs1,rqf1],[s,f]));
    
    qs2=1.1; qf2=1.4;  rqs2=1.1; rqf2=1.4;
    [s,f]=regular_JointPos(qs2,qf2);
    assert(isequal([rqs2,rqf2],[s,f]));
    
    qs3=-2; qf3=2; rqs3=-2+2*pi; rqf3=2;
    [s,f]=regular_JointPos(qs3,qf3);
    assert(isequal([rqs3,rqf3],[s,f]));
    
    qs4=pi; qf4=-pi; rqs4=pi; rqf4=pi;
    [s,f]=regular_JointPos(qs4,qf4);
    assert(isequal([rqs4,rqf4],[s,f]));
    
    qs5=0; qf5=2*pi; rqs5=0; rqf5=0;
    [s,f]=regular_JointPos(qs5,qf5);
    %disp([s,f])
    assert(isequal([rqs5,rqf5],[s,f]));
    
    qs6=-pi+0.1; qf6=2*pi-0.1; rqs6=pi+0.1; rqf6=2*pi-0.1;
    [s,f]=regular_JointPos(qs6,qf6);
    %disp([s,f])
    assert(isequal([rqs6,rqf6],[s,f]));
    
    qs7=-7*pi+0.1; qf7=4*pi-0.1; rqs7=pi+0.1; rqf7=2*pi-0.1;
    [s,f]=regular_JointPos(qs7,qf7);
    %disp([s,f])
    assert(sum(abs([rqs7,rqf7]-[s,f]))<0.00001);
    
    qs8=1.5; qf8=5; rqs8=1.5; rqf8=5-2*pi;
    [s,f]=regular_JointPos(qs8,qf8);
    %disp([s,f])
    assert(sum(abs([rqs8,rqf8]-[s,f]))<0.00001);
    
    [s,f]=regular_JointPos([qs1,qs2,qs3,qs4,qs5,qs6],[qf1,qf2,qf3,qf4,qf5,qf6]);
    assert(isequal([rqs1,rqs2,rqs3,rqs4,rqs5,rqs6;rqf1,rqf2,rqf3,rqf4,rqf5,rqf6],[s;f]));
end

