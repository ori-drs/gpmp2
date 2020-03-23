close all;
clear all;
clc;

figure(1);
h = subplot(1,2,1);

figure(2)

cell_size = 0.1;

% get X-Y coordinates
grid_rows = 100;
grid_cols = 100;
grid_z = 100;
grid_corner_x = 0 + (grid_cols-1)*0.1;
grid_corner_y = 0 + (grid_rows-1)*0.1;
grid_corner_z = 0 + (grid_z-1)*0.1;
grid_X = 0 : cell_size : grid_corner_x;
grid_Y = 0 : cell_size : grid_corner_y;
grid_Z = 0 : cell_size : grid_corner_z;

idx = [0, 20, 30, 40];
[x, y, z] = ind2sub([100,100,100],idx);

figure(1)
h = subplot(1,2,2);
scatter3([0,20,30,40,50] , [0,20,30,40,50], [0,20,30,40,50]);
colorbar;
xlabel('X/m');
ylabel('Y/m');
zlabel('Z/m');
axis equal
axis([0-cell_size/2, grid_corner_x+cell_size/2, ...
    0-cell_size/2, grid_corner_y+cell_size/2, ...
    0-cell_size/2, grid_corner_z+cell_size/2])