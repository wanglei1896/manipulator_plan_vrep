%% along path�滮����������
% ������е�������յ��ĩ��λ�ã���ξ���
% ���룺 
%    path: ·���ĵ�����[xseq; 
%                      yseq; 
%                      zseq] 
function result = input_ap(path, spacenum)
    result.pStart = [];
    result.qStart = [];
    result.path = regular_path(path, spacenum);
    result.spacenum = spacenum;
end

