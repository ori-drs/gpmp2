clear all;
clc;

map_size = [96, 96, 96];
num_repeats = 10000;
time_results = zeros(num_repeats,1);

% Set up static matrix with random values
mat = zeros(map_size);
s = RandStream('mlfg6331_64'); 
y = datasample(s,1:96^3,96^3,'Replace',true);
mat(y) = y./100;

% Set up an object matrix we want to superpose on top 
object_size = [15,15,96];
obj_matrix = ones(object_size)./10;


predicted_coord = [10, 10, 10]; % Bottom left coord of the position we want

for r = 1:num_repeats
    
    t_start = tic;
    
    % Initialise final mat as the starting mat
    final_mat = mat;

    % Find voxels the object occupies globally
    rows_range  = predicted_coord(1):predicted_coord(1) + object_size(1) - 1;
    cols_range  = predicted_coord(2):predicted_coord(2) + object_size(2) - 1;
    z_range     = predicted_coord(3):predicted_coord(3) + object_size(3) - 1;

    flip_rows_range = map_size(1) + 1 - rows_range;

    % Find mask for voxels within occupied within the workspace
    valid_row_mask  = flip_rows_range    >= 1 & flip_rows_range   <= map_size(1);
    valid_col_mask  = cols_range    >= 1 & cols_range   <= map_size(2);
    valid_z_mask    = z_range       >= 1 & z_range      <= map_size(3);

    % Mask the static matrix
    flip_rows_range  = flip_rows_range(valid_row_mask);
    cols_range  = cols_range(valid_col_mask);
    z_range     = z_range(valid_z_mask);

    static_matrix_slice    = final_mat(flip_rows_range, cols_range, z_range);

    % Mask the object matrix
    new_matrix_rows_range  = 1:object_size(1);
    new_matrix_rows_range  = new_matrix_rows_range(valid_row_mask);
    new_matrix_cols_range  = 1:object_size(2);
    new_matrix_cols_range  = new_matrix_cols_range(valid_col_mask);
    new_matrix_z_range     = 1:object_size(3);
    new_matrix_z_range     = new_matrix_z_range(valid_z_mask);

    new_matrix_slice       = obj_matrix(new_matrix_rows_range, new_matrix_cols_range, new_matrix_z_range);

    % Generate the composite SDF
    final_mat(flip_rows_range, cols_range , z_range) = min(static_matrix_slice, new_matrix_slice);

    t_end = toc(t_start);

    time_results(r) = t_end;
end


disp("Max: " + num2str(max(time_results)*1000) + " ms");
disp("Mean: " + num2str(mean(time_results)*1000) + " ms");
disp("Median: " + num2str(median(time_results)*1000) + " ms");
disp("Min: " + num2str(min(time_results)*1000) + " ms");

devs_away_from_mean = (max(time_results)-mean(time_results))/std(time_results);
disp("Max value is " + num2str(devs_away_from_mean) + " stds away from mean");

find(time_results==max(time_results))



histogram(time_results*1000, 10000);
set(gca,'TickDir','out'); % The only oth
xlabel('Time (ms)');
ylabel('Instances');