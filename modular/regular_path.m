% 此函数用于规范化路径点序列，使各点间变得等距离
function result = regular_path(path, spacenum)
    nPoint = size(path,2);
    delta_vecs = diff(path')';
    fraglen = [];
    for delta_vec = delta_vecs
        fraglen = [fraglen, norm(delta_vec)];
    end
    inter_odo = sum(fraglen)/spacenum;
    result = path(:,1);
    rest = 0;
    for i = 1:nPoint-1
        len = fraglen(i)+rest;
        if len>inter_odo
            b=inter_odo-rest;
            result=[result, path(:,i)+delta_vecs(:,i)*(b/fraglen(i))];
            len=len-inter_odo;
            if len>inter_odo
                n=floor(len/inter_odo);
                rate=inter_odo/fraglen(i);
                points=[];
                for j=1:n
                    points=[points,result(:,end)+delta_vecs(:,i)*rate];
                end
                result = [result, points];
                rest=len-n*inter_odo;
            else
                rest=len;
            end
        else
            rest=len;
        end
    end
    if size(result,2)==spacenum  %少一个点
        result = [result, path(:,end)];
    elseif size(result,2)>spacenum
        result(:,end)=path(:,end);
    end
end

