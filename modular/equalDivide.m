% 将长度为length+1的数组均匀分为group_num份
% 前(group_num-1)组长度相等，最后一组为余数
% 将相邻两组的下标作为结果返回，最后一组返回其自身
% group_num+1组仅返回最后一个值，作为哨兵

function index = equalDivide(length, group_num, i)
    assert(length>0 && group_num>0 && i>0 && i<=group_num+1)
    if i==group_num+1
       index = length+1;
       return;
    end
    remain = mod(length,group_num);
    % step1：get num
    if remain==0 
       num=length/group_num;       
    else
        if group_num==2
            num=(length+1)/2;
        else
            num=floor(length/(group_num-1));
        end
    end
    % step2：get index
    if i<group_num-1
        index = (1:(num*2+1))+(i-1)*num;
    else
        index = (i-1)*num+1:length+1;
    end
end

