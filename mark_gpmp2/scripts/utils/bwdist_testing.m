%% Testing 

matrix_size = 300;
occupancy_ratio = 0.01;
repeats = 10;

mat_range = 10:10:500;
occ_r_range = 0:0.05:1;

results = zeros(length(occ_r_range), length(mat_range));

for j = 1:length(mat_range)
    for i = 1:length(occ_r_range)
        results(i,j) = time_sdf(mat_range(j), occ_r_range(i));
    end
end





%%

[X,Y] = meshgrid(mat_range,occ_r_range);


figure(1); hold on;
surf(X,Y, results)
% colorbar;
view(3);
title('Performance of bwdist', 'FontSize', 14);
ylabel('Occupancy Ratio', 'FontSize', 12);
xlabel('Matrix Size', 'FontSize', 12);
zlabel('Time (s)', 'FontSize', 12);

%%


function mean_t = time_sdf(matrix_size, occupancy_ratio)
    s = RandStream('mlfg6331_64'); 
    repeats = 10;
    num_occupied_points = round((matrix_size^3)*occupancy_ratio);
    all_t = zeros(repeats, 1);
    for i = 1:repeats
        inds = randsample(s, matrix_size^3, num_occupied_points, true);
        map = zeros(matrix_size,matrix_size,matrix_size);
        map(inds) = 1;
        tic;
        map_dist = bwdist(map);
        all_t(i) = toc;
    end
    
    mean_t = mean(all_t);
end