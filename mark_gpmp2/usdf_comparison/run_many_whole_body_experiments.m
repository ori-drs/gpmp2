clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
total_time_sec = 5;
delta_t = 0.2;
interp_multiplier = 0;

cost_sigma = 0.05;
epsilon_dist = 0.1;    
limit_v = false;
limit_x = false;
cell_size = 0.04;
env_size = 64;
origin = [-1.28,-1.28,-1.28];
total_time_step = round(total_time_sec/delta_t);


%% Tasks

num_confs = 18;
confs = rand(num_confs, 8);
confs(:, 1:8) = (confs(:, 1:8) .* [2, 2, 2*pi, 0.69, 2.617, 5.584, 3.14, 5.584]) - ... 
                        [1, 1, pi, 0, 0, 3.665, 1.221, 3.665]; % Random numbers between -1 and 1 for x,y and 0-1 for z
          
combos = nchoosek(1:num_confs,2);
num_combos = size(combos,1);

%% Environment

% Obstacle def
num_obstacles = 100;
total_experiments = 2*num_obstacles*num_combos;

obstacle_set = rand(num_obstacles,6);
obstacle_set(:, 1:3) = (obstacle_set(:, 1:3) .* [2, 2, 2]) - [1,1,0]; % Random numbers between -1 and 1 for x,y and 0-1 for z

results_log = zeros(total_experiments, 6); % [index, signed(1)/unsigned(0) error, collisions, iterations]

for config_ind = 1:num_combos
    for e = 1:num_obstacles
        disp("Progress: " + num2str((e-1)*2 + (config_ind-1) * 2 * num_obstacles)  + "/" + num2str(total_experiments) );

        % Create the environment
        env = loadComparisonEnvironment(env_size, cell_size, origin);
        env.add_object(obstacle_set(e, 1:3), obstacle_set(e, 4:6));
        env.updateMap();

        %% Task

        % Setup problem
        start_pos_conf = confs(combos(config_ind, 1), :);
        end_pos_conf = confs(combos(config_ind, 2), :);

        start_pos = gtsam.Pose2(start_pos_conf(1), start_pos_conf(2), start_pos_conf(3));
        start_conf = start_pos_conf(4:end)';
        end_pos = gtsam.Pose2(end_pos_conf(1), end_pos_conf(2), end_pos_conf(3));
        end_conf = end_pos_conf(4:end)';


        problem_setup = HSRProblemSetup(gpmp2.Pose2Vector(start_pos, start_conf), gpmp2.Pose2Vector(end_pos, end_conf), total_time_sec, delta_t, ...
                                             cost_sigma, epsilon_dist, interp_multiplier, ...
                                             limit_x, limit_v);             

        evaluation_setup = HSRProblemSetup(gpmp2.Pose2Vector(start_pos, start_conf), gpmp2.Pose2Vector(end_pos, end_conf), total_time_sec, delta_t, ...
                                             cost_sigma, epsilon_dist, 20, ...
                                             limit_x, limit_v);   

        init_values = gpmp2.initPose2VectorTrajStraightLine(start_pos, start_conf, end_pos, end_conf, total_time_step);

        %% Planning

        usdf = env.getUSDF();
        hsr_planner_usdf = HSRGraphMaintainer(usdf, problem_setup);
        [usdf_result, ~, usdf_iterations] = hsr_planner_usdf.optimize(init_values);

        sdf = env.getSDF();
        hsr_planner_sdf = HSRGraphMaintainer(sdf, problem_setup);
        [sdf_result, ~, sdf_iterations] = hsr_planner_sdf.optimize(init_values);

        evaluation_planner = HSRGraphMaintainer(sdf, evaluation_setup);

        sdf_error = evaluation_planner.error(sdf_result);
        usdf_error = evaluation_planner.error(usdf_result);


        %% Analysis

        % How many collisions?
        sdf_collision_timeline = zeros(total_time_step + 1);
        for t = 0:total_time_step
            key_pos = gtsam.symbol('x', t);
            sdf_conf = gpmp2.atPose2VectorValues(key_pos, sdf_result);
            sdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(sdf_conf);
            sdf_num_collisions = sum(sdf_collision_timeline, 'all'); 
        end

        usdf_collision_timeline = zeros(total_time_step + 1);
        for t = 0:total_time_step
            key_pos = gtsam.symbol('x', t);
            usdf_conf = gpmp2.atPose2VectorValues(key_pos, usdf_result);
            usdf_collision_timeline(t+1) = evaluation_planner.collisionCheck(usdf_conf);
            usdf_num_collisions = sum(usdf_collision_timeline, 'all'); 
        end


%         disp("Error: " + num2str(error) + sprintf('\t') + "Iterations: " + num2str(iterations) +  sprintf('\t')  + "Collisions: " + num2str(num_collisions));
        results_log(1 + (e-1)*2 + (config_ind-1) * 2 * num_obstacles, :) = [config_ind, e, 0, usdf_error, double(usdf_num_collisions), double(usdf_iterations)];
        results_log(2 + (e-1)*2 + (config_ind-1) * 2 * num_obstacles, :) = [config_ind, e, 1, sdf_error, double(sdf_num_collisions), double(sdf_iterations)];
    end
end

save('/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/whole_body_results_lambda0_01', ...
'cell_size', ...
'combos',  ...
'confs',  ...
'cost_sigma', ... 
'delta_t', ...
'env_size',  ...
'epsilon_dist',  ...
'interp_multiplier', ... 
'limit_v',  ...
'limit_x',  ...
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
writematrix(results_log,'/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/whole_body_results_lambda0_01.csv');


% %% Visualisations
% 
% % Plot simulation
% 
% dataset = env.queryEnv();
% [X, Y, Z] = getEnvironmentMesh(dataset);
% figure(1); hold on; cla;
% lab_axis_lims = [-1.5 1.5 -1.5 1.5 -0.5 1];
% set(gcf,'Position',[1350 500 1200 1400]);
% axis(lab_axis_lims); 
% grid on; 
% view(3);
% xlabel('x'); ylabel('y'); zlabel('z');
% 
% for t = 0:total_time_step
% cla;
% h1 = plot3DEnvironment(dataset.map, X, Y, Z);
% 
% key_pos = gtsam.symbol('x', t);
% conf = result.atVector(key_pos);
% 
% h1 = plot3DEnvironment(dataset, X, Y, Z);
% 
% static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);
% 
% pause(0.05);
% end
% 
% 
% 
% % 
% % obs_factor = gpmp2.ObstacleSDFFactorArm(...
% %     key_pos, problem_setup.arm, sdf, problem_setup.cost_sigma, ...
% %     problem_setup.epsilon_dist);
% 
% %         if any(obs_factor.spheresInCollision(conf))
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
% %         else
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
% %         end
% 



% %% Restesting
% config_ind = 8;
% e = 50;
% 
% 
% % Create the environment
% env = loadComparisonEnvironment(env_size, cell_size, origin);
% env.add_object(obstacle_set(e, 1:3), obstacle_set(e, 4:6));
% env.updateMap();
% 
% % Task
% 
% % Setup problem
% start_conf = setPandaConf(confs(combos(config_ind, 1)));
% end_conf = setPandaConf(confs(combos(config_ind, 2)));
% 
% problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
%                                      cost_sigma, epsilon_dist, interp_multiplier, ...
%                                      limit_x, limit_v, base_pos);             
% 
% init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);
% 
% % Planning
% 
% usdf = env.getUSDF();
% panda_planner_usdf = pandaGraphMaintainer(usdf, problem_setup);
% [usdf_result, usdf_error, usdf_iterations] = panda_planner_usdf.optimize(init_values);
% 
% sdf = env.getSDF();
% panda_planner_sdf = pandaGraphMaintainer(sdf, problem_setup);
% [sdf_result, sdf_error, sdf_iterations] = panda_planner_sdf.optimize(init_values);
% 
% usdf_error_sdf = panda_planner_sdf.error(usdf_result);
% 
% sdf_error_usdf = panda_planner_usdf.error(usdf_result);
% 
% % Analysis
% 
% % How many collisions?
% sdf_collision_timeline = zeros(total_time_step + 1);
% for t = 0:total_time_step
%     key_pos = gtsam.symbol('x', t);
%     sdf_conf = sdf_result.atVector(key_pos);
%     sdf_collision_timeline(t+1) = panda_planner_usdf.collisionCheck(sdf_conf);
%     sdf_num_collisions = sum(sdf_collision_timeline, 'all'); 
% end
% 
% usdf_collision_timeline = zeros(total_time_step + 1);
% for t = 0:total_time_step
%     key_pos = gtsam.symbol('x', t);
%     usdf_conf = usdf_result.atVector(key_pos);
%     usdf_collision_timeline(t+1) = panda_planner_usdf.collisionCheck(usdf_conf);
%     usdf_num_collisions = sum(usdf_collision_timeline, 'all'); 
% end
% 
% 
% 
% % Plot simulation
% 
% dataset = env.queryEnv();
% [X, Y, Z] = getEnvironmentMesh(dataset);
% figure(1); hold on; cla;
% lab_axis_lims = [-1.5 1.5 -1.5 1.5 -0.5 1];
% set(gcf,'Position',[1350 500 1200 1400]);
% axis(lab_axis_lims); 
% grid on; 
% view(3);
% xlabel('x'); ylabel('y'); zlabel('z');
% 
% for t = 0:total_time_step
% cla;
% h1 = plot3DEnvironment(dataset.map, X, Y, Z);
% 
% key_pos = gtsam.symbol('x', t);
% conf = sdf_result.atVector(key_pos);
% 
% h1 = plot3DEnvironment(dataset, X, Y, Z);
% 
% static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);
% 
% pause(0.1);
% end
% 
% 
% 
% % 
% % obs_factor = gpmp2.ObstacleSDFFactorArm(...
% %     key_pos, problem_setup.arm, sdf, problem_setup.cost_sigma, ...
% %     problem_setup.epsilon_dist);
% 
% %         if any(obs_factor.spheresInCollision(conf))
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
% %         else
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
% %         end
% 
% 
