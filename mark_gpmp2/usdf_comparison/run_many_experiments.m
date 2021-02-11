clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
total_time_sec = 5;
delta_t = 0.2;
interp_multiplier = 10;
cost_sigma = 0.05;
epsilon_dist = 0.1;    
limit_v = false;
limit_x = false;
cell_size = 0.04;
env_size = 64;
origin = [-1.28,-1.28,-1.28];
base_pos = [0, 0, 0];
total_time_step = round(total_time_sec/delta_t);


%% Tasks

confs = ["ready", "right_ready", "behind", "left", "left_forward_shelf", "right_forward_shelf", "right_forward_shelf"....
            "in_shelf","top_shelf","left_in_shelf","table_forward","table_right","table_left","pickup_right", ...
            "pickup_left","table_forward_horz","table_left_horz","table_right_horz"];

num_confs = length(confs);
          
combos = nchoosek(1:num_confs,2);
num_combos = size(combos,1);

%% Environment

% Obstacle def
num_obstacles = 100;
total_experiments = 2*num_obstacles*num_combos;

obstacle_set = rand(num_obstacles,6);
obstacle_set(:, 1:3) = (obstacle_set(:, 1:3) .* [2, 2, 1]) - [1,1,0]; % Random numbers between -1 and 1 for x,y and 0-1 for z

results_log = zeros(total_experiments, 6); % [index, signed(1)/unsigned(0) error, collisions, iterations]

for config_ind = 1:num_combos
    for sdf_bool = 0:1
        for e = 1:num_obstacles
            disp("Progress: " + num2str(sdf_bool*num_obstacles + e + (config_ind-1) * 2 * num_obstacles)  + "/" + num2str(total_experiments) );
            % Create the environment
            env = loadComparisonEnvironment(env_size, cell_size, origin);
            env.add_object(obstacle_set(e, 1:3), obstacle_set(e, 4:6));
            env.updateMap();

            if boolean(sdf_bool)
                sdf = env.getSDF();
            else
                sdf = env.getUSDF();
            end

            %% Task

            % Setup problem
            start_conf = setPandaConf(confs(combos(config_ind, 1)));
            end_conf = setPandaConf(confs(combos(config_ind, 2)));


            problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                                 cost_sigma, epsilon_dist, interp_multiplier, ...
                                                 limit_x, limit_v, base_pos);             


            %% Planning
            init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

            panda_planner = pandaGraphMaintainer(sdf, problem_setup);

            [result, error, iterations] = panda_planner.optimize(init_values);

            %% Analysis

            % How many collisions?
            collision_timeline = zeros(total_time_step + 1);
            for t = 0:total_time_step
                key_pos = gtsam.symbol('x', t);
                conf = result.atVector(key_pos);
                collision_timeline(t+1) = panda_planner.collisionCheck(conf);
                num_collisions = sum(collision_timeline, 'all');
            end

    %         disp("Error: " + num2str(error) + sprintf('\t') + "Iterations: " + num2str(iterations) +  sprintf('\t')  + "Collisions: " + num2str(num_collisions));
            results_log(sdf_bool*num_obstacles + e + (config_ind-1) * 2 * num_obstacles, :) = [config_ind, e, sdf_bool, error, double(num_collisions), double(iterations)];
        end % obstacle

    end
end

% save('/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/30600results.mat', ...
% 'base_pos', ...
% 'cell_size', ...
% 'combos',  ...
% 'confs',  ...
% 'cost_sigma', ... 
% 'delta_t', ...
% 'env_size',  ...
% 'epsilon_dist',  ...
% 'interp_multiplier', ... 
% 'limit_v',  ...
% 'limit_x',  ...
% 'num_combos',  ...
% 'num_confs',  ...
% 'num_obstacles',  ...
% 'obstacle_set',  ...
% 'origin',  ...
% 'problem_setup', ... 
% 'results_log',  ...
% 'total_experiments', ... 
% 'total_time_sec',  ...
% 'total_time_step')
% 
% writematrix(results_log,'/home/mark/installs/gpmp2/mark_gpmp2/usdf_comparison/30600results.csv');


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
