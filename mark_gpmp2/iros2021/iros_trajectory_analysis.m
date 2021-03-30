import gtsam.*
import gpmp2.*
%% Bag params 

nh = ros.Node('matlab');

%% Shelf experiment
% predict_planning_results_filename1 = '/home/mark/iros2021/exp9_with_obs.csv';
% predict_planning_results_filename1 = '/home/mark/iros2021/exp9_with_obs_new.csv';

shelf_file = '/home/mark/iros2021/hardware_experiments/shelf_experiment.csv';

T = readtable(shelf_file);

%  Convert to matrix
arr = table2array(T); 
times = unique(arr(:,1));

% t = times(1);
t = times(55);
% t = times(57);
t = times(59);

actual_traj_inds = find(arr(:, 1) == t);
actual_traj = arr(actual_traj_inds, :);

% Lets get the actual trajectory execute 
actual_traj_inds = find(arr(:, 1) == 1000);
actual_traj = arr(actual_traj_inds, :);

disp_traj = displayHSRTrajectory(nh);
disp_traj.publish(actual_traj);

%% Floor experiment

floor_file = '/home/mark/iros2021/hardware_experiments/floor2.csv';



T = readtable(floor_file);

%  Convert to matrix
arr = table2array(T); 
times = unique(arr(:,1));

% t = times(10);

actual_traj_inds = find(arr(:, 1) == t);
actual_traj = arr(actual_traj_inds, :);

% Lets get the actual trajectory execute 
actual_traj_inds = find(arr(:, 1) == 1000);
actual_traj = arr(actual_traj_inds, :);

disp_traj = displayHSRTrajectory(nh);
disp_traj.publish(actual_traj);

%% Floor experiment

sim_file = '/home/mark/iros2021/simulation/floor_without_obs.csv';



T = readtable(sim_file);

%  Convert to matrix
arr = table2array(T); 
times = unique(arr(:,1));

% t = times(10);

% actual_traj_inds = find(arr(:, 1) == t);
% actual_traj = arr(actual_traj_inds, :);

% Lets get the actual trajectory execute 
actual_traj_inds = find(arr(:, 1) == 1000);
actual_traj = arr(actual_traj_inds, :);

disp_traj = displayHSRTrajectory(nh);
disp_traj.publish(actual_traj);