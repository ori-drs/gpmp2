classdef movingEnvironment
    %MOVINGENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        t_start_moving
        v_or_t_end
        v_or_t_end_value
        block_starting_pos
        dataset
        block_velocity
        t_end_moving
        block_end_pos
    end
    
    methods
        function obj = movingEnvironment(t_start_moving, v_or_t_end, v_or_t_end_value, block_starting_pos, obs_size)
            %MOVINGENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here
%             block_starting_pos(2) = -block_starting_pos(2);
            
            obj.t_start_moving = t_start_moving;
            obj.v_or_t_end = v_or_t_end;
            obj.v_or_t_end_value = v_or_t_end_value;

            obj.block_starting_pos = block_starting_pos;
%             obj.block_starting_pos = [-0.50, 1.90];
%             obj.dataset.obs_size = [0.60, 0.80];
            obj.dataset.obs_size = obs_size;

            % Parameters for block trajectory
            if v_or_t_end == true % object velocity given
                obj.block_velocity = v_or_t_end_value;        
            elseif v_or_t_end == false % t_end given
                obj.t_end_moving = v_or_t_end_value;
                obj.block_end_pos = [2.50, 1.50];
                obj.block_velocity = (block_end_pos - block_starting_pos)/(t_end_moving-t_start_moving);
            end
            
            % params
            obj.dataset.cols = 300;
            obj.dataset.rows = 300;
%             obj.dataset.origin_x = 0;
%             obj.dataset.origin_y = 0;
            obj.dataset.origin_x = -1;
            obj.dataset.origin_y = -1;
            obj.dataset.cell_size = 0.01;
            obj.dataset.origin_point2 = gtsam.Point2(obj.dataset.origin_x, obj.dataset.origin_y);       
            
        end
        
        function val = queryEnv(obj, query_t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % map
            obj.dataset.map = zeros(obj.dataset.rows, obj.dataset.cols);
            
            if query_t<= obj.t_start_moving
               t_moved = 0;
            else
               t_moved = query_t - obj.t_start_moving;
            end
            % position is start plus v * t
            x = obj.block_starting_pos(1) + t_moved * obj.block_velocity(1);
            y = obj.block_starting_pos(2) + t_moved * obj.block_velocity(2);

            if obj.v_or_t_end == false && query_t<= obj.v_or_t_end_value
                x = obj.block_end_pos(1);
                y = obj.block_end_pos(2);
            end

            % obstacles
            obs_pos_to_add = round([x - obj.dataset.origin_x, ...
                                    -y + obj.dataset.origin_y]/obj.dataset.cell_size) ...
                                                + [0, obj.dataset.rows];

            obj.dataset.map = add_obstacle(obs_pos_to_add, round(obj.dataset.obs_size/obj.dataset.cell_size), obj.dataset.map);
        
            obj.dataset.map = flip(obj.dataset.map);
            
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField2D(obj.dataset.map, obj.dataset.cell_size);
            obj.dataset.sdf = gpmp2.PlanarSDF(obj.dataset.origin_point2, obj.dataset.cell_size, obj.dataset.field);
            obj.dataset.obs_pose = [x,y];
            
            val = obj.dataset;
        end
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks)

half_width = floor((obj_size(1)-1)/2);
half_height = floor((obj_size(2)-1)/2);

% occupency grid
X = position(1)-half_width : position(1)+half_width;
Y = position(2)-half_height : position(2)+half_height;

y_inds_in_range = Y>=1 & Y<= size(map,1);
Y = Y(y_inds_in_range);

x_inds_in_range = X>=1 & X<= size(map,2);
X = X(x_inds_in_range);

map(Y, X) = ones(size(Y, 2), size(X, 2)); 

end