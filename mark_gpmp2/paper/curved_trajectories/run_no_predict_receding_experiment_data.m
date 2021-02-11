clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

plot_graphs = false;
total_time_sec = 3;
delta_t = 0.1;
interp_multiplier = 1;
cost_sigma = 0.05;
epsilon_dist = 0.3;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
env_size = 200;
origin = [-1,-1,-1];
base_pos = [-0.05, 0, 0.4];

total_time_step = round(total_time_sec/delta_t);

% Setup problem
start_conf = setPandaConf('table_forward');
end_conf = setPandaConf('table_forward');



        
%% This is repeated for each experiment
% radii = 2;
% speeds = 2.0;

radii = 1:1:10;
speeds = 0.2:0.2:2.0;

num_trials = length(radii) * length(speeds);

collision_results = zeros(num_trials,3);


trial_num = 1;
for r = radii
    for speed = speeds
        disp("Progress: " + num2str(trial_num) + "/" + num2str(num_trials));        
         
        trajCalculator = recedingCurvedTrajectory(speed, r);

        problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, ...
                                         limit_x, limit_v, base_pos);

        init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

        env = curvedEnvironment(env_size, cell_size, origin);
        start_sdf = env.getSDF();
        panda_planner = recedingPandaPlanner(start_sdf, problem_setup);
        tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);

        sdfs  = cell(total_time_step, 1);
        for i = 1:total_time_step+1
            sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                        cell_size, ...
                                                        env_size, ...
                                                        env_size, ...
                                                        env_size);
        end

        pause(1);

        env.updateMap(trajCalculator.getPosition(0));

        % First optimisation
        result = panda_planner.optimize(init_values);
        %%%

        t_step = 1;

        num_steps_in_collision = 0;

        for time_steps = 0:ceil(pi * r / (speed * delta_t)) 

            block_pos = trajCalculator.getPosition(t_step * delta_t);

            env.updateMap(block_pos);

            tracker.update(t_step * delta_t, env.dataset.map);

            curr_conf = result.atVector(gtsam.symbol('x', 1));

            panda_planner.graph.replace(0, gtsam.PriorFactorVector(gtsam.symbol('x', 0), result.atVector(gtsam.symbol('x', 1)), problem_setup.pose_fix_model));
            panda_planner.graph.replace(1, gtsam.PriorFactorVector(gtsam.symbol('v', 0), result.atVector(gtsam.symbol('v', 1)), problem_setup.pose_fix_model));

            field = tracker.predict_composite_sdf(0);

            for t = 0:total_time_step

                for z = 1:env_size
                    sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                end

                panda_planner.update_sdf(t, sdfs{t+1});
            end

            init_values = gpmp2.initArmTrajStraightLine(curr_conf, end_conf, total_time_step);
            result = panda_planner.optimize(init_values);


            key_pos = gtsam.symbol('x',0);
            obs_factor = gpmp2.ObstacleSDFFactorArm(...
                key_pos, problem_setup.arm, env.getSDF(), problem_setup.cost_sigma, ...
                0);
            
            num_steps_in_collision =  num_steps_in_collision + sum(any(obs_factor.evaluateError(result.atVector(key_pos))));

            t_step = t_step + 1;
        end 
        
        
        % Record data
        collision_results(trial_num, :) = [speed, r, num_steps_in_collision];
        trial_num = trial_num + 1;

    end % speed
end % radius


col_bools = collision_results(:,3);
col_bools = col_bools>0;
col_bools = reshape(col_bools, length(speeds),[]);
col_bools = flipud(col_bools);
yvalues = {'0.2','0.4','0.6','0.8','1.0','1.2','1.4','1.6','1.8','2.0'};
xvalues = {'1.0', '2.0', '3.0', '4.0', '5.0', '6.0', '7.0', '8.0', '9.0', '10.0'};
h = heatmap(xvalues,flip(yvalues),int8(col_bools),'CellLabelColor','none');
h.XLabel = 'Radius (m)';
h.YLabel = 'Obstacle Speed (m/s)';
h.ColorbarVisible = 'off';
h.FontSize = 16;