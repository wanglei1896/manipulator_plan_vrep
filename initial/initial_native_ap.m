%% ��matlab���س�ʼ����along path�滮����
global  outputData ...
        inputData ...
        optimLog ...       %�Ż���־���㷨�Ż������в�������Ϣ�����ڷ�����
        model           %��е�۵��˶�ѧģ��

%%% ��ʼ��
model = model_ur5();
optimLog = optimLog_ap(2);   %�Ż��м�����
inputData = input_ap([0.3, 0.6;    %�����·��
                      0.4,   0;
                        0,   0], optimLog.group_num*10); %����·���淶����Ĳ�������
outputData = output_multiSeg();

%%% Ĭ��
inputData.pStart = eye(4);
inputData.pStart(1:3,4) = inputData.path(:,1);
inputData.qStart = model.ikunc(inputData.pStart);
outputData.spacenum = optimLog.group_num*10;