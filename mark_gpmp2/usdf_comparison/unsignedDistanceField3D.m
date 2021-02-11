function map_dist = unsignedDistanceField3D(ground_truth_map, cell_size)

% regularize unknow area to open area
map = (ground_truth_map > 0.75);

% get signed distance from map and inverse map
map_dist = bwdist(map);

% metric
map_dist = map_dist * cell_size;
map_dist = double(map_dist);

% limit inf
if isinf(map_dist(1,1))
    map_dist = ones(size(map_dist)) * 1000;
end

end

