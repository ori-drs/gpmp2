function [X, Y, Z] = getEnvironmentMesh(dataset)
%GETENVIRONMENTMESH Summary of this function goes here
%   Detailed explanation goes here

    x1 = dataset.origin_x;
    y1 = dataset.origin_y;
    z1 = dataset.origin_z;    

    % get X-Y coordinates
    grid_rows = dataset.rows;
    grid_cols = dataset.cols;
    grid_z = dataset.z;

    grid_corner_x = x1 + (grid_cols-1)*dataset.cell_size;
    grid_corner_y = y1 + (grid_rows-1)*dataset.cell_size;
    grid_corner_z = z1 + (grid_z-1)*dataset.cell_size;

    grid_X = x1 : dataset.cell_size : grid_corner_x;
    grid_Y = y1 : dataset.cell_size : grid_corner_y;
    grid_Z = z1 : dataset.cell_size : grid_corner_z;

    [X,Y,Z] = meshgrid(grid_X,grid_Y,grid_Z);
    
end

