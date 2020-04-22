% 测试GJK函数的时间消耗

function GJK_test_time()
    V1 = [0.0000   -1.0515   -1.7013;
         0.0000   -1.0515    1.7013;
         0.0000    1.0515    1.7013;
         0.0000    1.0515   -1.7013;
        -1.0515   -1.7013         0;
        -1.0515    1.7013         0];
    
    V2 = [-0.7071   -0.7071         0;
         0.7071   -0.7071         0;
         0.7071    0.7071         0;
        -0.7071    0.7071         0;
         0.0000         0   -1.0000;
         0.0000         0    1.0000];

    S1Obj.XData=V1(:,1);
    S1Obj.YData=V1(:,2);
    S1Obj.ZData=V1(:,3);
    S2Obj.XData=V2(:,1);
    S2Obj.YData=V2(:,2);
    S2Obj.ZData=V2(:,3);
    iterationsAllowed=10;
    tic
    for i=1:10000
        GJK(S1Obj,S2Obj,iterationsAllowed);
    end
    toc
end

