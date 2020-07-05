% 格式化CoppeliaSim端规划过程图
for i=1:size(fromVrepData.snapshot,4)
    imshow(fromVrepData.snapshot(:,:,:,i))
end