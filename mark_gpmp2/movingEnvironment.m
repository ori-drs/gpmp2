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
        function obj = movingEnvironment(t_start_moving, v_or_t_end, v_or_t_end_value)
            %MOVINGENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here
            obj.t_start_moving = t_start_moving;
            obj.v_or_t_end = v_or_t_end;
            obj.v_or_t_end_value = v_or_t_end_value;

            obj.block_starting_pos = [0.50, 0.50];
            obj.dataset.obs_size = [0.60, 0.80];

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
            obs_pos_to_add = round([-y + obj.dataset.origin_y,...
                                    x - obj.dataset.origin_x]/obj.dataset.cell_size) ...
                                + [obj.dataset.rows,0];

            obj.dataset.map = add_obstacle(obs_pos_to_add, flip(round(obj.dataset.obs_size/obj.dataset.cell_size)), obj.dataset.map);

            % signed distance field
            

            obj.dataset.field  = gpmp2.signedDistanceField2D(obj.dataset.map, obj.dataset.cell_size);
            obj.dataset.sdf = gpmp2.PlanarSDF(obj.dataset.origin_point2, obj.dataset.cell_size, obj.dataset.field);
            obj.dataset.obs_pose = [x,y];
            
            val = obj.dataset;
        end
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((obj_size(1)-1)/2);
half_size_col = floor((obj_size(2)-1)/2);

% occupency grid
Y = position(1)-half_size_row : position(1)+half_size_row;
X = position(2)-half_size_col : position(2)+half_size_col;

y_inds_in_range = Y>=1 & Y<= size(map,1);
Y = Y(y_inds_in_range);

x_inds_in_range = X>=1 & X<= size(map,2);
X = X(x_inds_in_range);

map(Y, X) = ones(size(Y, 2), size(X, 2)); 

% landmarks
if nargin == 7
    for x = Y(1)-1 : 4 : Y(end)-1
        y = X(1)-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = X(end)-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = X(1)+3 : 4 : X(end)-5
        x = Y(1)-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = Y(end)-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
end

end

function center = get_center(x,y,dataset)

center = [y - dataset.origin_y, x - dataset.origin_x]./dataset.cell_size;

end

function dim = get_dim(w,h,dataset)

dim = [h, w]./dataset.cell_size;

end