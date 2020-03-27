function dataset = generateMovingScene(scenario, t_start_moving, v_or_t_end, v_or_t_end_value, query_t)

    if strcmp(scenario, 'OneObstacleDataset')

    %     block_starting_pos = [-0.50, 1.90];
        block_starting_pos = [0.50, 0.50];
    %     block_starting_pos = [0, 0];
        block_size = [0.60, 0.80];

        % Parameters for block trajectory
        if v_or_t_end == true % object velocity given
            block_velocity = v_or_t_end_value;        
        elseif v_or_t_end == false % t_end given
            t_end_moving = v_or_t_end_value;
            block_end_pos = [2.50, 1.50];
            block_velocity = (block_end_pos - block_starting_pos)/(t_end_moving-t_start_moving);
        end

        if query_t<= t_start_moving
           t_moved = 0;
        else
           t_moved = query_t - t_start_moving;
        end
        % position is start plus v * t
        x = block_starting_pos(1) + t_moved * block_velocity(1);
        y = block_starting_pos(2) + t_moved * block_velocity(2);

        if v_or_t_end == false && query_t<= v_or_t_end_value
            x = block_end_pos(1);
            y = block_end_pos(2);
        end

        % params
        dataset.cols = 300;
        dataset.rows = 300;
        dataset.origin_x = -1;
        dataset.origin_y = -1;
        dataset.cell_size = 0.01;
        
        % map
        dataset.map = zeros(dataset.rows, dataset.cols);
        
        % obstacles
        obs_pos_to_add = round([-y + dataset.origin_y,...
                                x - dataset.origin_x]/dataset.cell_size) ...
                            + [dataset.rows,0];

        dataset.map = add_obstacle(obs_pos_to_add, flip(round(block_size/dataset.cell_size)), dataset.map);

        % signed distance field
        dataset.origin_point2 = gtsam.Point2(dataset.origin_x, dataset.origin_y);

        dataset.field  = gpmp2.signedDistanceField2D(dataset.map, dataset.cell_size);
        dataset.sdf = gpmp2.PlanarSDF(dataset.origin_point2, dataset.cell_size, dataset.field);
        dataset.obs_size = block_size;
        dataset.obs_pose = [x,y];

    elseif strcmp(scenario, 'Static')
        block_starting_pos = [0.60, 0.45];
        block_size = [0.60, 0.80];
        
        x = block_starting_pos(1);
        y = block_starting_pos(2);

        % params
        dataset.cols = 300;
        dataset.rows = 300;
        dataset.origin_x = -1;
        dataset.origin_y = -1;
        dataset.cell_size = 0.01;
        
        % map
        dataset.map = zeros(dataset.rows, dataset.cols);

        % obstacles
        obs_pos_to_add = round([-y + dataset.origin_y,...
                                x - dataset.origin_x]/dataset.cell_size) ...
                            + [dataset.rows,0];    
        dataset.map = add_obstacle(obs_pos_to_add, flip(round(block_size/dataset.cell_size)), dataset.map);
                
        % signed distance field
        dataset.origin_point2 = gtsam.Point2(dataset.origin_x, dataset.origin_y);

        dataset.field  = gpmp2.signedDistanceField2D(dataset.map, dataset.cell_size);
        dataset.sdf = gpmp2.PlanarSDF(dataset.origin_point2, dataset.cell_size, dataset.field);
        dataset.obs_size = block_size;
        dataset.obs_pose = block_starting_pos;
    else
        disp("No such scenarios");
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
