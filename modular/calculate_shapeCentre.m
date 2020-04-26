% 用于计算三维几何体的形状中心
% input: 每列为一个顶点
%   [p1x p2x p3x ...;
%    p1y p2y p3y ...;
%    p1z p2z p3z ...]

function result = calculate_shapeCentre(vexData)
    s=sum(vexData,2);
    result=s/size(vexData,2);
end

