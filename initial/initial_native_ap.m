%% ��matlab���س�ʼ����along path�滮����
global  outputData ...
        inputData ...
        optimLog ...       %�Ż���־���㷨�Ż������в�������Ϣ�����ڷ�����
        model           %��е�۵��˶�ѧģ��

%%% ��ʼ��
model = model_ur5();
optimLog = optimLog_ap(6);   %�Ż��м�����
inputData = input_ap([0.2,  0.1;    %�����·��
                      0.4,  0.3;
                        0, -0.1], optimLog.group_num*10); %����·���淶����Ĳ�������
outputData = output_multiSeg();

%%% Ĭ��
inputData.pStart = [1 0 0 0.2;
                    0 1 0 0.4;
                    0 0 1 0;
                    0 0 0 1];
inputData.qStart = model.ikunc(inputData.pStart);
outputData.spacenum = optimLog.group_num*10;