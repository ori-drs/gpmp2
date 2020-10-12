% 
close all
clear

import gtsam.*
import gpmp2.*

dataset.cols = 64;
dataset.rows = 64;
dataset.z = 64;
dataset.origin_x = 0;
dataset.origin_y = 0;
dataset.origin_z = 0;
dataset.cell_size = 1;

% map
dataset.map = zeros(dataset.rows, dataset.cols, dataset.z);

% obstacles
dataset.corner_idx = [];
[dataset.map, dataset.corner_idx] = add_obstacle([2 3 6], [3 5 11], dataset.map, dataset.corner_idx);

field = signedDistanceField3D(dataset.map, dataset.cell_size);

dlmwrite('gpu_voxels_test_3_5_11.txt',field)
% A(1, :, :) = [1 1 1; 1 1 1]
% dlmwrite('test.txt',field)

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

