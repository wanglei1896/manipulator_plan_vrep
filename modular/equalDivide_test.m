function equalDivide_test()
    assert(isequal(equalDivide(20,4,1),1:11));
    assert(isequal(equalDivide(20,4,2),6:16));
    assert(isequal(equalDivide(20,4,3),11:21));
    assert(isequal(equalDivide(20,4,4),16:21));
    
    assert(isequal(equalDivide(13,4,1),1:9));
    assert(isequal(equalDivide(13,4,2),5:13));
    assert(isequal(equalDivide(13,4,3),9:14));
    assert(isequal(equalDivide(13,4,4),13:14));
    
    assert(isequal(equalDivide(12,2,1),1:13));
    assert(isequal(equalDivide(12,2,2),7:13));
    
    assert(isequal(equalDivide(13,2,1),1:14));
    assert(isequal(equalDivide(13,2,2),8:14));
    
    disp('well done!')
end

