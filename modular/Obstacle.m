classdef (Abstract) Obstacle < EnvironmentEntity
    %OBSTACLE �˴���ʾ�йش����ժҪ
    %   �˴���ʾ��ϸ˵��
    
    properties
        surface_point_set
    end
    
    methods
        function obj = Obstacle()
        end
        
        draw(obj)
        isLapped(obj, point_set)
        isLappedWithEntity(obj, entity)
    end
end

