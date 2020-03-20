function dataset = generate3Dmovingdataset(dataset_str, shift, num_steps)
%GENERATE3DDATASET enerate 3D dataset evidence grid
%
%   Usage: dataset = GENERATE3DDATASET(dataset_str)
%   @dataset_str       dataset string, existing datasets:
%                      'WAMDeskDataset'
%
%   Dataset Format:
%   dataset.map        ground truth evidence grid
%   dataset.rows       number of rows (x)
%   dataset.cols       number of cols (y)
%   dataset.z          number of depth (z)
%   dataset.origin_x   origin of map x
%   dataset.origin_y   origin of map y
%   dataset.origin_z   origin of map z
%   dataset.cell_size  cell size
%   dataset.corner_idx corner index to visualize edges

import gtsam.Point3
import gpmp2.signedDistanceField3D
import gpmp2.SignedDistanceField

if nargin == 1  
  shift = [0,0,0];
  num_steps = 18;
end

% small dataset for WAM
if strcmp(dataset_str, 'WAMDeskDataset')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.5;
    dataset.origin_y = -1.5;
    dataset.origin_z = -1.5;
    dataset.cell_size = 0.01;
    
    % map
    dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);
    
    % obstacles
    dataset.corner_idx = [];
    
    % Add table
%     [dataset.map, dataset.corner_idx] = add_obstacle([170 220 130], [140, 60, 5], dataset.map, dataset.corner_idx);
%     [dataset.map, dataset.corner_idx] = add_obstacle([105 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
%     [dataset.map, dataset.corner_idx] = add_obstacle([235 195 90], [10, 10, 80], dataset.map, dataset.corner_idx);
%     [dataset.map, dataset.corner_idx] = add_obstacle([105 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);
%     [dataset.map, dataset.corner_idx] = add_obstacle([235 245 90], [10, 10, 80], dataset.map, dataset.corner_idx);

%     % Add sides of the cabinet
%     [dataset.map, dataset.corner_idx] = add_obstacle([250 190 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
%     [dataset.map, dataset.corner_idx] = add_obstacle([250 90 145], [60, 5, 190], dataset.map, dataset.corner_idx);   
%    
%     % Add extra wall partition
%     [dataset.map, dataset.corner_idx] = add_obstacle([200 190 145], [40, 5, 190], dataset.map, dataset.corner_idx);   
%  
    % Add the four shelves
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 240], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 190], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 140], [60, 100, 5], dataset.map, dataset.corner_idx);
    [dataset.map, dataset.corner_idx] = add_obstacle([250 140 90], [60, 100, 5], dataset.map, dataset.corner_idx);

% small dataset for WAM
elseif strcmp(dataset_str, 'MovingBox')
    % params
    dataset.cols = 300;
    dataset.rows = 300;
    dataset.z = 300;
    dataset.origin_x = -1.5;
    dataset.origin_y = -1.5;
    dataset.origin_z = -1.5;
    dataset.origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
    dataset.origin_point3 = Point3(dataset.origin');

    dataset.cell_size = 0.01;
    
    dataset.maps = [];
    dataset.corner_idxs = [];

    
    for i = 0 : num_steps
        % map
        map = zeros(dataset.rows, dataset.cols, dataset.z);

        % obstacles
        corner_idx = [];

    %     % Add sides of the box
        [map, corner_idx] = add_obstacle([220 191 165]+ i*shift, [50, 5, 55], map, corner_idx);   
        [map, corner_idx] = add_obstacle([220 89 165]+i*shift, [50, 5, 55], map, corner_idx);   

        % Add the two shelves of the box
        [map, corner_idx] = add_obstacle([220 140 190]+i*shift, [50, 100, 5], map, corner_idx);
        [map, corner_idx] = add_obstacle([220 140 140]+i*shift, [50, 100, 5], map, corner_idx);
        
        
        dataset.maps{i+1} = map;
        dataset.corner_idxs{i+1} = corner_idx;
        dataset.fields{i+1} = signedDistanceField3D(map, dataset.cell_size);

        
        % init sdf
        sdf = SignedDistanceField(dataset.origin_point3, dataset.cell_size, size(dataset.fields{i+1}, 1), ...
            size(dataset.fields{i+1}, 2), size(dataset.fields{i+1}, 3));
        for z = 1:size(dataset.fields{i+1}, 3)
            sdf.initFieldData(z-1, dataset.fields{i+1}(:,:,z)');
        end
        
        dataset.sdfs{i+1} = sdf;
        
    end
        
    
% no such dataset
else
    error('No such dataset exist');
end

end

function [map, corner] = add_obstacle(position, size, map, corner)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);
half_size_z = floor((size(3)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col, ...
    position(3)-half_size_z   : position(3)+half_size_z) ...
    = ones(2*half_size_row+1, 2*half_size_col+1, 2*half_size_z+1); 

% corner
corner = [corner; ...
   [position(1)-half_size_row , position(1)+half_size_row, ...
    position(2)-half_size_col , position(2)+half_size_col,...
    position(3)-half_size_z   , position(3)+half_size_z]];


end


