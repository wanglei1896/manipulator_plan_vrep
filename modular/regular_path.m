% 此函数用于规范化路径点序列，使各点间变得等路程
% 输入：任意路径序列, 以及采样段数（输出是多少段的路径）
% 输出：规范化后的等路程路径序列，两点间的路程指的是沿原有路径序列连成的折线走的路程
% 条件1：路径数据结构格式：[xseq;yseq;zseq], 也可以是二维([xseq;yseq])或一维
% 

function result = regular_path(path, spacenum)
    nPoint = size(path,2);
    delta_vecs = diff(path')';
    lens = zeros(1,nPoint-1); %每段的长度
    lenCur = zeros(1,nPoint); %每段的累积长度
    for ii=2:nPoint
        lens(ii-1) = norm(delta_vecs(:,ii-1));
        lenCur(ii) = lenCur(ii-1)+lens(ii-1); 
    end
    result = zeros(size(path,1),spacenum+1);
    fast();
    
    %% 设某一折线段是由一点p在参数alpha的变化下的运动出来的，alpha属于[0,1]
    % 给定一alpha值，求p点位置
    function point = getPinAlpha(alpha)
        
    end
    
    %% 设某一折线段是由一点p（从起点至终点，不走回头路）运动出来的
    % 求当运动路程为distance时p点的位置
    function point = getPinDistance(distance)
        %为减轻计算量，用向量的1范数作长度度量
        
    end

    %% 尽可能快的算法
    function fast()
        inter_odo = lenCur(nPoint)/spacenum;
        result(:,1) = path(:,1);
        result_i = 1; %result数组的游标,指向最后添加的元素的位置
        rest = 0;
        for i = 1:nPoint-1 %i游标表示path中的第几段
            len = lens(i)+rest;
            if len>inter_odo
                b=inter_odo-rest;
                result_i=result_i+1;
                result(:,result_i)=path(:,i)+delta_vecs(:,i)*(b/lens(i));
                len=len-inter_odo;
                if len>inter_odo
                    n=floor(len/inter_odo);
                    rest=len-n*inter_odo;
                    rate=inter_odo/lens(i);
                    while n>0
                        result_i=result_i+1;
                        result(:,result_i)=result(:,result_i-1)+delta_vecs(:,i)*rate;
                        n=n-1;
                    end
                else
                    rest=len;
                end
            else
                rest=len;
            end
        end
        if result_i==spacenum  %少一个点
            result(:,end) = path(:,end);
        end
    end
end

