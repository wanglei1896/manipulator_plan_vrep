% ���룺��������Ļ���ֵ
% ������淶����Ļ���ֵ����Χ(-pi,pi*2)����������֮��ľ���ֵ<=pi
% ����1���������������ֵ
% ����2��ͨ��+2n*pi��-2n*pi���淶����
% ����3�����Լ�����������һ������һ��Ϊ6��
% ����4�����������ʱ��ȡ�������飬��(-pi+0.1)��(2*pi-0.1)��ӦΪ(pi+0.1)��(2*pi-0.1)

function [regqStart, regqFinal] = regular_JointPos(qStart, qFinal)
    for i=1:length(qStart)
        qs=qStart(i);qf=qFinal(i);
        qs=constrict(qs);
        qf=constrict(qf);
        if abs(qs-qf)>pi
            if(qs<qf)
                if(qs<0)
                    qs=qs+2*pi;
                elseif(qs<pi)
                    qf=qf-2*pi;
                end
            else
                if(qf<0)
                    qf=qf+2*pi;
                elseif(qf<pi)
                    qs=qs-2*pi;
                end
            end
        end
        regqStart(i)=qs;
        regqFinal(i)=qf;
    end
    
    function rq=constrict(q)
        while true
            if q>=pi*2
                q=q-pi*2;
            elseif q<=-pi
                q=q+pi*2;
            else
                break;
            end
        end
        rq=q;
    end
end

