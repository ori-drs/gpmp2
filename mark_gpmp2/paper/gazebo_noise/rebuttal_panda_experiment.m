clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

num_repeats = 50;
% variances = [0, 0.1, 0.01, 0.001, 0.0001, 0.00001];
variances = [0.1, 0.01, 0.001, 0.0];
speeds = [1.2, 1.4, 1.6, 1.8, 2.0];

scene = "tables"; % "bookshelf" 
obstacle = "hsrb";

plot_graphs = false;
total_time_sec = 3;
delta_t = 0.1;
interp_multiplier = 1;
cost_sigma = 0.05;
epsilon_dist = 0.3;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
% env_size = 64;
env_sizes = [64, 96, 128];
origin = [-1,-1,-1];

base_pos = [-0.05, 0, 0.4];

% Setup problem
% start_conf = setPandaConf('right_ready');
% % end_conf = setPandaConf('table_forward');
% end_conf = setPandaConf('table_right');
conf_combo = [["right_ready","table_forward"]; ...
            ["right_ready","table_left"]; ...
            ["right_ready","table_right"]; ...
            ["pickup_right","table_forward"];...
            ["pickup_right","table_left"]; ...
            ["pickup_right","table_right"]; ...
            ["pickup_left","table_forward"]; ...
            ["pickup_left","table_left"]; ...
            ["pickup_left","table_right"]
            ];
        
% Setup ROS interactions
node = ros.Node('/matlab_node');

isFakePublisher = true;

traj_publisher = trajectoryPublisher(delta_t, isFakePublisher);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
start_sim_pub = simulationPublisher(node, obstacle);

velocity_Msg = rosmessage('std_msgs/Float32');
velocity_Msg.Data = 1.2;

total_time_step = round(total_time_sec/delta_t);

num_algos = 4;
num_conf_combos = size(conf_combo,1);
num_speeds = length(speeds);
num_experiments = num_algos * num_repeats * length(variances) * num_speeds * num_conf_combos * length(env_sizes);
experiment_results = zeros(num_experiments, 8);
%% This is repeated for each experiment


exp_num = 1;
for conf_row = 1:num_conf_combos
    start_conf = setPandaConf(conf_combo(conf_row, 1));
    end_conf = setPandaConf(conf_combo(conf_row, 2));
    problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos);             

    init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    for env_size = env_sizes
        for algo = [0,1,2,3]
            for speed = speeds
                velocity_Msg.Data = speed;
                
                for var = variances
                    for r = 1:num_repeats

                        try
                            t_update = 0;
                            t_step = 0;
                            num_replans = 0;

                            env = rebuttalLiveEnvironment(node, env_size, cell_size, origin, obstacle, scene, var);
                            env.add_table_static_scene();

                            start_sdf = env.getSDF();

                            panda_planner = pandaPlanner(start_sdf, problem_setup);

                            tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                                            epsilon_dist, cell_size, env.dataset.origin_point3);

                            traj_publisher.goToConfig(start_conf);

                            sdfs  = cell(total_time_step, 1);
                            for i = 1:total_time_step+1
                                sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                                            cell_size, ...
                                                                            env_size, ...
                                                                            env_size, ...
                                                                            env_size);
                            end

                %             pause(4);
                            pause(1);


                            % All is reset - start the simulation

                            start_sim_pub.send(velocity_Msg); 

                            t_start = tic;

                            env.updateMap();

                            % First optimisation
                            result = panda_planner.optimize(init_values);

                            % Execute
                            traj_publisher.publish(result, t_step);

                            last_lock = 0;

                            while t_update < total_time_sec

                                % Get latest conf and sdf to update
                                t_update = toc(t_start);
                                latest_msg = sub.LatestMessage;
                                curr_conf = latest_msg.Position(3:end);
                                curr_vel = latest_msg.Velocity(3:end);
                                env.updateMap();

                                t_step = floor(t_update/delta_t);


                                tracker.update(t_update, env.dataset.map);

                                for p = last_lock:t_step
                                    panda_planner.update_confs(p, ...
                                                                result.atVector(gtsam.symbol('x', p)), ...
                                                                result.atVector(gtsam.symbol('v', p)));
                                end
                                last_lock = t_step + 1;

                                if algo == 0 % Use prediction for each factor using composite method
                                    for t = t_step:total_time_step
                                        forward_t = (t-t_step)*delta_t;

                                        field = tracker.predict_composite_sdf(forward_t);

                                        for z = 1:env_size
                                            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                                        end

                                        panda_planner.update_sdf(t, sdfs{t+1});
                                    end
                                elseif algo == 1 % Use prediction for each factor using full calculation
                                    for t = t_step:total_time_step
                                        forward_t = (t-t_step)*delta_t;

                                        field = tracker.predict_field(t*delta_t);

                                        for z = 1:env_size
                                            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                                        end

                                        panda_planner.update_sdf(t, sdfs{t+1});
                                    end                        
                                elseif algo == 2 % Use composite method for execute and update
                                    field = tracker.predict_composite_sdf(0);

                                    for t = t_step:total_time_step
                                        for z = 1:env_size
                                            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                                        end
                                        panda_planner.update_sdf(t, sdfs{t+1});
                                    end
                                elseif algo == 3 % Use full method for execute and update
                                    field = tracker.predict_field(0);

                                    for t = t_step:total_time_step
                                        for z = 1:env_size
                                            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                                        end
                                        panda_planner.update_sdf(t, sdfs{t+1});
                                    end
                                end
                                result = panda_planner.optimize(init_values);

                                % Execute
                                if t_step < total_time_step -1
                                    traj_publisher.publish(result, t_step);
                                    num_replans = num_replans + 1;
                                end
                                t_update = toc(t_start);

                            end %while loop

                            [smoothness_cost, num_collisions] = pandaGroundTruthCollisionCheck(env_size, cell_size, problem_setup, result);
                            col_free_bool = num_collisions == 0;
                            experiment_results(exp_num, :) = [conf_row, algo, speed, var, col_free_bool, num_collisions, smoothness_cost, num_replans];
                        catch
                            disp("Encountered a problem. Skipping");
                            experiment_results(exp_num, :) = [conf_row, algo, speed, var, 1000, 1000, 1000, 1000];
                        end
                        disp("Progress: " + exp_num + "/" + num_experiments);
                        exp_num = exp_num + 1;
                    end % for each repeat
                    filename = "/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_noise/results/varied_results_" + num2str(exp_num-1) + ".csv";                    
                    writematrix(experiment_results,filename)
                end % for each variance
            end %speed
        end % for each algo
    end %env_sizes
end % conf
writematrix(experiment_results,'/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_noise/results/varied_result_finished.csv')