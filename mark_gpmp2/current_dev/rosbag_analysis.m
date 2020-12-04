import gtsam.*
import gpmp2.*
%% Bag params 

delta_t = 0.2;
nh = ros.Node('matlab2');


%% Batch 1 Predict
predict_bagname1 = '/home/mark/friday_data/6/predict/2020-10-30-15-35-13.bag';
predict_planning_results_filename1 = '/home/mark/friday_data/6/predict/prediction_planning_results.csv';

T = readtable(predict_planning_results_filename1);

%  Convert to matrix
arr = table2array(T); 

middle_t = 2.5072;
last_t = 4.5674;

start_arr = arr(arr(:, 1) == 0, :);
mid_arr = arr(abs(arr(:, 1) - middle_t) < 10^-4, :);
end_arr = arr(abs(arr(:, 1) - last_t) < 10^-4, :);

disp_traj = displaySavedTrajectory(nh, delta_t);
% disp_traj.publish(start_arr);
% disp_traj.publish(mid_arr);
disp_traj.publish(end_arr);

%% Batch 1 - Execute and update
bench_bagname1 = '/home/mark/friday_data/6/bench/2020-10-30-15-36-33.bag ';
exec_planning_results_filename1 = '/home/mark/friday_data/6/bench/exec_update_planning_results.csv';

T = readtable(exec_planning_results_filename1);

%  Convert to matrix
arr = table2array(T); 

middle_t = 2.5152;
last_t = 4.5778;

start_arr = arr(arr(:, 1) == 0, :);
mid_arr = arr(abs(arr(:, 1) - middle_t) < 10^-4, :);
end_arr = arr(abs(arr(:, 1) - last_t) < 10^-4, :);

disp_traj = displaySavedTrajectory(nh, delta_t);
% disp_traj.publish(start_arr);
% disp_traj.publish(mid_arr);
disp_traj.publish(end_arr);

%% Batch 2 Predict
predict_bagname2 = '/home/mark/friday_data/3/predict/2020-10-30-14-33-44.bag';
predict_planning_results_filename2 = '/home/mark/friday_data/3/predict/prediction_planning_results.csv';

T = readtable(predict_planning_results_filename2);

middle_t = 2.5152;
last_t = 4.5160;

arr = table2array(T);

start_arr = arr(arr(:, 1) == 0, :);
mid_arr = arr(abs(arr(:, 1) - middle_t) < 10^-4, :);
end_arr = arr(abs(arr(:, 1) - last_t) < 10^-4, :);

disp_traj = displaySavedTrajectory(nh, delta_t);

% disp_traj.publish(start_arr);
% disp_traj.publish(mid_arr);
disp_traj.publish(end_arr);

%% Batch 2 - Execute and update - AHHH I overwrote this
bench_bagname2 = '/home/mark/friday_data/3/bench/2020-10-30-14-34-28.bag';
exec_planning_results_filename2 = '/home/mark/friday_data/3/bench/exec_update_planning_results.csv';

T = readtable(exec_planning_results_filename2);

middle_t = 2.6102;
last_t = 4.5160;

arr = table2array(T);

start_arr = arr(arr(:, 1) == 0, :);
mid_arr = arr(abs(arr(:, 1) - middle_t) < 10^-4, :);
end_arr = arr(abs(arr(:, 1) - last_t) < 10^-4, :);

disp_traj = displaySavedTrajectory(nh, delta_t);

% disp_traj.publish(start_arr);
disp_traj.publish(mid_arr);
% % disp_traj.publish(end_arr);
