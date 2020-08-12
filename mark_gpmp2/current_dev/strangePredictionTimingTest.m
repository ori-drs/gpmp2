clear all;
clc;

map_size = [96, 96, 96];
num_repeats = 1000;
time_results = zeros(num_repeats,1);

% Set up static scene matrix with random values
mat = zeros(map_size);
s = RandStream('mlfg6331_64'); 
y = datasample(s,1:96^3,96^3,'Replace',true);
mat(y) = y./100;

% Set up an object SDF 
sdf_size = [15,15,96];
obj_sdf = ones(sdf_size)./10;


predicted_coord = [10, 10, 10]; % Bottom left coord of the predicted position

rows_range_base  = -(0:sdf_size(1) - 1);
cols_range_base  = 0:sdf_size(2) - 1;
z_range_base     = 0:sdf_size(3) - 1;

flip_rows_range  = zeros(1, sdf_size(1));
cols_range  = zeros(1, sdf_size(2));
z_range     = zeros(1, sdf_size(3));

valid_row_mask = false(1, sdf_size(1));
valid_col_mask = false(1, sdf_size(2));
valid_z_mask = false(1, sdf_size(3));

clear 'new_sdf_rows_range' 'new_sdf_cols_range' 'new_sdf_z_range' 'new_sdf_slice' ...
    'static_sdf_slice' 'new_sdf_slice' 'predicted_sdf';
clear functions;   
% clear classes;        

for r = 1:num_repeats
    
    t_start = tic;
    
    % Initialise composite SDF as the static SDF
    predicted_sdf = mat;

    % Find voxels the object occupies globally
    flip_rows_range  = rows_range_base + predicted_coord(1) + map_size(1) + 1;
    cols_range  = cols_range_base + predicted_coord(2);
    z_range     = z_range_base + predicted_coord(3);

    % Find mask for voxels within occupied within the workspace
    valid_row_mask  = flip_rows_range    >= 1 & flip_rows_range   <= map_size(1);
    valid_col_mask  = cols_range    >= 1 & cols_range   <= map_size(2);
    valid_z_mask    = z_range       >= 1 & z_range      <= map_size(3);

    % Mask the static SDF
    flip_rows_range  = flip_rows_range(valid_row_mask);
    cols_range  = cols_range(valid_col_mask);
    z_range     = z_range(valid_z_mask);

    static_sdf_slice    = predicted_sdf(flip_rows_range, cols_range, z_range);

    % Mask the object SDF
    new_sdf_rows_range  = 1:sdf_size(1);
    new_sdf_rows_range  = new_sdf_rows_range(valid_row_mask);
    new_sdf_cols_range  = 1:sdf_size(2);
    new_sdf_cols_range  = new_sdf_cols_range(valid_col_mask);
    new_sdf_z_range     = 1:sdf_size(3);
    new_sdf_z_range     = new_sdf_z_range(valid_z_mask);

    new_sdf_slice       = obj_sdf(new_sdf_rows_range, new_sdf_cols_range, new_sdf_z_range);

    % Generate the composite SDF
    predicted_sdf(flip_rows_range, cols_range , z_range) = min(static_sdf_slice, new_sdf_slice);

    t_end = toc(t_start);

    time_results(r) = t_end;
    
    clear 'new_sdf_rows_range' 'new_sdf_cols_range' 'new_sdf_z_range' 'new_sdf_slice' ...
        'static_sdf_slice' 'new_sdf_slice' 'predicted_sdf';
    clear functions;        
%     clear classes;        
end


disp("Max: " + num2str(max(time_results)*1000) + " ms");
disp("Mean: " + num2str(mean(time_results)*1000) + " ms");
disp("Median: " + num2str(median(time_results)*1000) + " ms");
disp("Min: " + num2str(min(time_results)*1000) + " ms");

devs_away_from_mean = (max(time_results)-mean(time_results))/std(time_results);
disp("Max value is " + num2str(devs_away_from_mean) + " stds away from mean");

histogram(time_results*1000, 10000);
set(gca,'TickDir','out'); % The only oth
xlabel('Time (ms)');
ylabel('Instances');