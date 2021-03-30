clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
total_time_sec = 2;
interp_multiplier = 2;
cost_sigma = 0.01;
epsilon_dist = 0.1;  

cell_size = 0.04;
env_size = 125;
origin = [-5,-5];

% Generate random set of confs
num_confs = 18;
num_obstacles = 100;

confs = rand(num_confs,3);
% confs = (confs .* [5, 5, 3.14]) - [5, 5, 0]; 
confs = (confs .* [5, 5, 0]) - [5, 5, -pi/2]; 

combos = nchoosek(1:num_confs,2);
num_combos = size(combos,1);


% Generate set of obstacles
obstacle_set = rand(num_obstacles,4); % [x, y, width, height]
obstacle_set(:, 1:2) = (obstacle_set(:, 1:2) .* [5, 5]) - [5,5]; 
obstacle_set(:, 3:4) = (obstacle_set(:, 3:4) .* [3, 3]); 

delta_t_range = 0.1:0.1:0.5;
%% Environment

total_experiments = num_obstacles*num_combos*length(delta_t_range);

results_log = zeros(total_experiments, 10); % [index, signed(1)/unsigned(0), obs_area, delta_t, usdf_error, usdf_collisions, usdf_iterations, 
                                            %                                                   sdf_error, sdf_collisions, sdf_iterations]

for t_ind = 1:length(delta_t_range)
    
    delta_t = delta_t_range(t_ind);
    total_time_step = round(total_time_sec/delta_t);

    for config_ind = 1:num_combos
        
        start_pose = confs(combos(config_ind, 1), :);
        end_pose = confs(combos(config_ind, 2), :);    
        start_pose = gtsam.Pose2(start_pose(1), start_pose(2), start_pose(3));
        end_pose = gtsam.Pose2(end_pose(1), end_pose(2), end_pose(3));
            
        for e = 1:num_obstacles
            
            exp_id = (t_ind - 1) * num_obstacles * num_combos + (config_ind - 1) * num_obstacles + e;
            disp("Progress: " + num2str(exp_id)  + "/" + num2str(total_experiments) );

            % Create the environment
            env = load2DComparisonEnvironment(env_size, cell_size, origin);
            env.add_object([-5,-2.5], [5,0.4]);
            env.add_object([0,-2.5], [5,0.4]);
            env.add_object([-2.5,-5], [0.4,5]);
            env.add_object([-2.5, 0], [0.4,5]);
            env.add_object(obstacle_set(e, 1:2), obstacle_set(e, 3:4));
            env.updateMap();

            % Task
            problem_setup = mobileProblemSetup2D(start_pose, end_pose, total_time_sec, delta_t, ...
                                                 cost_sigma, epsilon_dist, interp_multiplier);

            evaluation_setup = mobileProblemSetup2D(start_pose, end_pose, total_time_sec, delta_t, ...
                                                 cost_sigma, epsilon_dist, 10);

            % Planning
            init_values = init2DMobileValues(problem_setup);

            usdf = env.getUSDF();
            planner_usdf = graphMaintainer2D(usdf, problem_setup);
            [usdf_result, ~, usdf_iterations] = planner_usdf.optimize(init_values);

            sdf = env.getSDF();
            planner_sdf = graphMaintainer2D(sdf, problem_setup);
            [sdf_result, ~, sdf_iterations] = planner_sdf.optimize(init_values);

            evaluation_planner = graphMaintainer2D(sdf, evaluation_setup);

            sdf_error = evaluation_planner.error(sdf_result);
            usdf_error = evaluation_planner.error(usdf_result);


            % Analysis
            sdf_collision_timeline = zeros(total_time_step + 1);
            for t = 0:total_time_step
                key_pos = gtsam.symbol('x', t);
                sdf_conf = sdf_result.atPose2(key_pos);
                sdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(sdf_conf);
                sdf_num_collisions = sum(sdf_collision_timeline, 'all'); 
            end

            usdf_collision_timeline = zeros(total_time_step + 1);
            for t = 0:total_time_step
                key_pos = gtsam.symbol('x', t);
                usdf_conf = usdf_result.atPose2(key_pos);
                usdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(usdf_conf);
                usdf_num_collisions = sum(usdf_collision_timeline, 'all'); 
            end

            obs_area = obstacle_set(e, 3) * obstacle_set(e,4);

            results_log(exp_id, :) = [config_ind, e, obs_area, delta_t, ...
                                    usdf_error, double(usdf_num_collisions), double(usdf_iterations), ...
                                    sdf_error, double(sdf_num_collisions), double(sdf_iterations)];

        end
    end
end


save('/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/2dresults_lambda0_01.mat', ...
'confs', ...
'delta_t_range', ...
'cell_size', ...
'combos',  ...
'confs',  ...
'cost_sigma', ... 
'delta_t', ...
'env_size',  ...
'epsilon_dist',  ...
'interp_multiplier', ... 
'num_combos',  ...
'num_confs',  ...
'num_obstacles',  ...
'obstacle_set',  ...
'origin',  ...
'problem_setup', ... 
'results_log',  ...
'total_experiments', ... 
'total_time_sec',  ...
'total_time_step')
% 
writematrix(results_log,'/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/2dresults_lambda0_01.csv');


%% Environment checking
% 10, 21, 22, 32	

e = 10;

env = load2DComparisonEnvironment(env_size, cell_size, origin);
env.add_object([-5,-2.5], [5,0.4]);
env.add_object([0,-2.5], [5,0.4]);
env.add_object([-2.5,-5], [0.4,5]);
env.add_object([-2.5, 0], [0.4,5]);
env.add_object(obstacle_set(e, 1:2), obstacle_set(e, 3:4));
env.updateMap(); 
dataset = env.queryEnv();

% plot_inter = 0;
% if plot_inter
%     total_plot_step = total_time_step * (plot_inter + 1);
%     plot_values = gpmp2.interpolatePose2Traj(result, problem_setup.Qc_model, problem_setup.delta_t, plot_inter, 0, problem_setup.total_time_step);
% else
%     total_plot_step = total_time_step;
%     plot_values = result;
% end

figure(4), hold on
gpmp2.plotEvidenceMap2D(permute(dataset.map, [2,1]), dataset.origin_x, dataset.origin_y, cell_size);
% for i=0:total_plot_step
%     p = plot_values.atPose2(symbol('x', i));
%     plotPlanarMobileBase(problem_setup.robot.fk_model(), p, [0.4 0.2], 'b', 1);
% end
hold off;
