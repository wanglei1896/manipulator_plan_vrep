tic,
assignment(1000000);
%testTrigono(1000000);
toc

function assignment(times)
    b=zeros(times,1);
    for i=1:times
        b(i)=pi;
    end
end

function testTrigono(times)
    for i=1:times
        sin(i);
    end
end