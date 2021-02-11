classdef curvedTrajectory < handle
    
    properties
        duration
        speed 
        e
        
        b
        a
        
    end
    
    methods
        function obj = curvedTrajectory(duration, speed, e)
            
            obj.e = e;
            obj.duration = duration;
            obj.speed = speed;
            
            obj.b = obj.speed * obj.duration / pi;
            obj.a = obj.speed * obj.duration / pi;
        end
        

                    
        function pos = getPosition(obj, t)
            
            if obj.e == 0
                % a = b = r/2
                a = obj.speed * obj.duration / pi;
                b = a;
                
                pos = [0.4 + obj.a*(1 - sin(pi * t/obj.duration)), ...
                        obj.b * cos(pi * t/obj.duration), ...
                        0];         
%             elseif ((obj.e > 0) && (obj.e < 1))
% 
%                 a = sin(t * pi/ obj.duration) * obj.speed * obj.duration / (pi * sqrt((1 - (obj.e * sin(t * pi/ obj.duration))^2)));
%                 b = a;                 
            end
                    
%                 s
%             obj.a = a;
%             obj.b = sqrt(4 * (obj.duration^2) * (obj.speed^2) - (obj.a ^ 2));
%             
%             
%             pos = [0.4 + obj.a*(1 - sin(pi * t/obj.duration)), ...
%                 obj.b * cos(pi * t/obj.duration), ...
%                 0];      
        end
    end
    
end

