function dataset = generate2Dmovingdataset(dataset_str, num_steps)
%GENERATE2DDATASET Generate 2D dataset evidence grid
%
%   Usage: dataset = GENERATE2DDATASET(dataset_str)
%   @dataset_str       dataset string, existing datasets:
%                      'OneObstacleDataset', 'TwoObstaclesDataset'
%
%   Output Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (y)
%   dataset.cols       number of cols (x)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.cell_size  cell size


% dataset 5: 1 obs dataset for 2D Arm obs avoid
if strcmp(dataset_str, 'OneObstacleDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.origin_x = -1;
    dataset.origin_y = -1;
    dataset.cell_size = 0.01;
    % map
    dataset.maps = [];
    x_pos_limits = [160, 50];
    y_pos_limits = [190, 150];
        
    for i = 0 : num_steps
        x = x_pos_limits(1) + i * (x_pos_limits(2)-x_pos_limits(1))/num_steps;
        y = y_pos_limits(1) + i * (y_pos_limits(2)-y_pos_limits(1))/num_steps;
        
        maps{i+1} = zeros(dataset.rows, dataset.cols);
        % obstacles
        maps{i+1} = add_obstacle([y,x], [60, 80], maps{i+1});
        % signed distance field
        origin_point2 = gtsam.Point2(dataset.origin_x, dataset.origin_y);

        field{i+1} = gpmp2.signedDistanceField2D(maps{i+1}, dataset.cell_size);
        sdf{i+1} = gpmp2.PlanarSDF(origin_point2, dataset.cell_size, field{i+1});

    end

    dataset.maps = maps;
    dataset.fields = field;
    dataset.sdfs = sdf;
    
% no such dataset
else
    error('No such dataset exist');
end

end

function [map, landmarks] = add_obstacle(position, size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col) ...
    = ones(2*half_size_row+1, 2*half_size_col+1); 

% landmarks
if nargin == 7
    for x = position(1)-half_size_row-1 : 4 : position(1)+half_size_row-1
        y = position(2)-half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = position(2)+half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = position(2)-half_size_col+3 : 4 : position(2)+half_size_col-5
        x = position(1)-half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = position(1)+half_size_row-1;
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
