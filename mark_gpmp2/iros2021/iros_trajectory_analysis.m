import gtsam.*
import gpmp2.*
%% Bag params 

nh = ros.Node('matlab');

%% Batch 1 Predict
% predict_planning_results_filename1 = '/home/mark/iros2021/exp9_with_obs.csv';
predict_planning_results_filename1 = '/home/mark/iros2021/exp9_without_obs.csv';

T = readtable(predict_planning_results_filename1);

%  Convert to matrix
arr = table2array(T); 

% Lets get the actual trajectory execute 
actual_traj_inds = find(arr(:, 1) == 1000);
actual_traj = arr(actual_traj_inds, :);

disp_traj = displayHSRTrajectory(nh);
disp_traj.publish(actual_traj);