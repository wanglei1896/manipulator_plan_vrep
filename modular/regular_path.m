% �˺������ڹ淶��·�������У�ʹ������õ�·��
% ���룺����·������, �Լ���������������Ƕ��ٶε�·����
% ������淶����ĵ�·��·�����У�������·��ָ������ԭ��·���������ɵ������ߵ�·��
% ����1��·�����ݽṹ��ʽ��[xseq;yseq;zseq], Ҳ�����Ƕ�ά([xseq;yseq])��һά
% 

function result = regular_path(path, spacenum)
    nPoint = size(path,2);
    delta_vecs = diff(path')';
    lens = zeros(1,nPoint-1); %ÿ�εĳ���
    lenCur = zeros(1,nPoint); %ÿ�ε��ۻ�����
    for ii=2:nPoint
        lens(ii-1) = norm(delta_vecs(:,ii-1));
        lenCur(ii) = lenCur(ii-1)+lens(ii-1); 
    end
    result = zeros(size(path,1),spacenum+1);
    fast();
    
    %% ��ĳһ���߶�����һ��p�ڲ���alpha�ı仯�µ��˶������ģ�alpha����[0,1]
    % ����һalphaֵ����p��λ��
    function point = getPinAlpha(alpha)
        
    end
    
    %% ��ĳһ���߶�����һ��p����������յ㣬���߻�ͷ·���˶�������
    % ���˶�·��Ϊdistanceʱp���λ��
    function point = getPinDistance(distance)
        %Ϊ�������������������1���������ȶ���
        
    end

    %% �����ܿ���㷨
    function fast()
        inter_odo = lenCur(nPoint)/spacenum;
        result(:,1) = path(:,1);
        result_i = 1; %result������α�,ָ�������ӵ�Ԫ�ص�λ��
        rest = 0;
        for i = 1:nPoint-1 %i�α��ʾpath�еĵڼ���
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
        if result_i==spacenum  %��һ����
            result(:,end) = path(:,end);
        end
    end
end

