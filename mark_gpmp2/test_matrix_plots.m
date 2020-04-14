clear all 
clc
close all

figure(1)

origin = [0,0,0];
grid_cols = 100;
grid_rows = 100;
cell_size = 0.01;
marker_size = 10;


field = zeros(100,100);
block_start = [20,20];
block_size = [10, 10];

for i = 0:block_size(1)
    for j = 1:block_size(2)
        field(block_start(1) + i, block_start(2) + j) = 1;
    end
end

block_pos_x = [block_start(1); block_start(1)+block_size(1); ...
    block_start(1)+block_size(1); block_start(1)];

block_pos_y = [block_start(2); block_start(2); ...
    block_start(2)+block_size(2); block_start(2)+block_size(2)];

c = [1];

block_pos_x = block_pos_x * cell_size;
block_pos_y = block_pos_y * cell_size;

patch(block_pos_x, ...
    block_pos_y, ...
    c);

patch(block_pos_x + 0.2, ...
    block_pos_y + 0.2, ...
    c,...
    'FaceAlpha', 0.5);

colorbar;


xlabel('X/m');
ylabel('Y/m');
axis equal

colormap(gray(100));
