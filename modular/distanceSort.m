% 贪心法重排序点数组，使得每两个点的距离尽可能短
% input:
%   [p1x, p1y, p1z, ...;
%    p2x, p2y, p2z, ...;
%    ...
%    pnx, pny, pnz, ...];
function result = distanceSort(input)
    result=input(1,:);
    remains=input(2:end,:);
    [psize, pdim]=size(input);
    for i=1:psize-1
        choose1=result(end,:);
        choose2=remains(1,:);
        dis=norm(choose1-choose2);
        choose2id=1;
        for j=1:size(remains,1)
        	if norm(choose1-remains(j,:))<dis
                dis=norm(choose1-remains(j,:));
                choose2=remains(j,:);
                choose2id=j;
            end
        end
        result=[result; choose2];
        if choose2id==1
            remains=remains(2:end,:);
        elseif choose2id==size(remains,1)
            remains=remains(1:end-1,:);
        else
            remains1=remains(1:choose2id-1,:);
            remains2=remains(choose2id+1:end,:);
            remains=[remains1; remains2];
        end
    end
    %{
    disp('before:')
    for i=1:size(input,1)-1
        disp(norm(input(i,:)-input(i+1,:)))
    end
    disp('after:')
    for i=1:size(result,1)-1
        disp(norm(result(i,:)-result(i+1,:)))
    end
    %}
end

