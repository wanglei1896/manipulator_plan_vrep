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
    tic
    for i=1:300000
        dist=openGJK(V1',V2');
    end
    toc
end

