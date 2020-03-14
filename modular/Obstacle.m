classdef (Abstract) Obstacle < EnvironmentEntity
    %OBSTACLE 此处显示有关此类的摘要
    %   此处显示详细说明
    
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

