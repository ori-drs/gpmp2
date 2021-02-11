classdef recedingCurvedTrajectory < handle
    
    properties
        duration
        speed 
        r
        
        b
        a
        
    end
    
    methods
        function obj = recedingCurvedTrajectory(speed, r)
            
            obj.r = r;
            obj.speed = speed;
            
            obj.b = r;
            obj.a = r;
        end
        

                    
        function pos = getPosition(obj, t)
            
            theta = obj.speed * t / obj.r;

            pos = [0.4 + obj.a*(1 - sin(theta)), ...
                    obj.b * cos(theta), ...
                    0];                      

        end
        
    end
    
end

