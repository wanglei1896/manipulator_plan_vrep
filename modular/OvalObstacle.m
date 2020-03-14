classdef OvalObstacle < Obstacle
    %EVAL 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        scale
        ratio
    end
    
    methods
        function obj = OvalObstacle(coordinate,scale,ratio)
            obj.coordinate = coordinate;
            obj.scale = scale;
            obj.ratio = ratio;
            obj.surface_point_set = obj.toSurfacePointSet;
        end
        
        function draw(obj)
            surface_point_set = obj.toSurfacePointSet;
            plot(surface_point_set(1,:),surface_point_set(2,:))
        end
        function flag = isLappedWithEntity(obj,entity)
            entity_point_set = entity.toSurfacePointSet;
            surface_point_set = obj.toSurfacePointSet;
            in=inpolygon(entity_point_set(1,:),entity_point_set(2,:),...
                        surface_point_set(1,:),surface_point_set(2,:));
            flag = any(any(in));
        end
        function flag = isLapped(obj,point_set)
            in=inpolygon(point_set(1,:),point_set(2,:),...
                        obj.surface_point_set(1,:),obj.surface_point_set(2,:));
            flag = any(any(in));
        end

        function result = toSurfacePointSet(obj)
            sample_rate = 20;
            theta = linspace(0,2*pi,sample_rate);
            x = cos(theta);
            y = sin(theta)*obj.ratio;
            result = [x;y]*obj.scale+obj.coordinate;
        end
    end
end

