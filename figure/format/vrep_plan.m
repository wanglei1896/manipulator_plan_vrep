% 格式化CoppeliaSim端规划过程图
plotVrepPlan();

function plotVrepPlan()
global fromVrepData
    num=size(fromVrepData.snapshot,4);
    index=4:4:num;
    
    for i=1:length(index)
        subplot(2,5,i),
        imshow(fromVrepData.snapshot(:,:,:,index(i))),
        xlabel(['t=',num2str(index(i)),'s'])
    end
end