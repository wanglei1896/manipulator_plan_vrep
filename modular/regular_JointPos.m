% 输入：两个任意的弧度值
% 输出：规范化后的弧度值，范围(-pi,pi*2)，且两弧度之差的绝对值<=pi
% 条件1：输入可以是向量值
% 条件2：通过+2n*pi或-2n*pi来规范弧度
% 条件3：可以假设输入向量一样长（一般为6）
% 条件4：当有两组解时，取更大那组，如(-pi+0.1)与(2*pi-0.1)解应为(pi+0.1)和(2*pi-0.1)

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

