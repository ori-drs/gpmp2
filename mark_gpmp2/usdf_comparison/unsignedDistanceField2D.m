function field = unsignedDistanceField2D(ground_truth_map, cell_size)
%SIGNEDDISTANCEFIELD2D 2D signed distance field
%   Given a ground truth 2D map defined in Matrix in 0-1,
%   calculate 2D signed distance field, which is defined as a matrix
%   map matrix and signed distance field matrix have the same resolution.
%
%   Usage: field = SIGNEDDISTANCEFIELD2D(ground_truth_map, cell_siz)
%   @map        evidence grid from dataset, map use 0 show open area, 1 show objects. 
%   @cell_size  cell sizeto given metric information
%
%   Output: 
%   @field      sdf, row is Y, col is X


% regularize unknow area to open area
map = (ground_truth_map > 0.75);

% get signed distance from map and inverse map
field = bwdist(map);

% metric
field = field * cell_size;
field = double(field);

% limit inf
if isinf(field(1,1))
    field = ones(size(field)) * 1000;
end

end

