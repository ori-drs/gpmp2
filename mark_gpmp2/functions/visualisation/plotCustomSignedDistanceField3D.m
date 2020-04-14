function h = plotCustomSignedDistanceField3D(field, origin, cell_size, epsilon_min, epsilon_max)
%PLOTSIGNEDDISTANCEFIELD3D plot 3D SignedDistanceField
%
%   Usage: plotCustomSignedDistanceField3D(field, origin, cell_size, epsilon_dist)
%   @field                  field 3D matrix
%   @origin                 origin of the map
%   @cell_size              cell size
%   @epsilon_dist           optional plot obstacle safety distance, default = 0
%   @marker_size            marker size, default = 0

% if nargin < 4
%     epsilon_min = 0; 
%     epsilon_max = 0;
% end

% get X-Y coordinates
grid_rows = size(field, 1);
grid_cols = size(field, 2);
grid_z = size(field, 3);
grid_corner_x = origin(1) + (grid_cols-1)*cell_size;
grid_corner_y = origin(2) + (grid_rows-1)*cell_size;
grid_corner_z = origin(3) + (grid_z-1)*cell_size;
grid_X = origin(1) : cell_size : grid_corner_x;
grid_Y = origin(2) : cell_size : grid_corner_y;
grid_Z = origin(3) : cell_size : grid_corner_z;

if nargin < 4
    idx = find(field<0);
else
    idx = find(field<=epsilon_max & field>=epsilon_min);
end

% idx = find(field<=epsilon_max && field>=epsilon_min);
[x, y, z] = ind2sub(size(field),idx);
c = field(idx);

figure(2);
h = scatter3(grid_X(y), grid_Y(x), grid_Z(z), 2, c);
colorbar;
xlabel('X/m');
ylabel('Y/m');
zlabel('Z/m');
axis equal
axis([origin(1)-cell_size/2, grid_corner_x+cell_size/2, ...
    origin(2)-cell_size/2, grid_corner_y+cell_size/2, ...
    origin(3)-cell_size/2, grid_corner_z+cell_size/2])


end
