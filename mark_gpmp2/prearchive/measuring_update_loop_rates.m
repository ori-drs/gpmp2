clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

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
env_size = 96;
origin = [-1,-1,-1];

base_pos = [0, 0, 0.4];

start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');

total_time_step = round(total_time_sec/delta_t);
problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, ...
                                         limit_x, limit_v, base_pos);   
init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);                                     
% Setup ROS interactions
node = ros.Node('/matlab_node');

traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
start_sim_pub = simulationPublisher(node, obstacle);

velocity_Msg = rosmessage('std_msgs/Float32');
% velocity_Msg.Data = 1.2;
velocity_Msg.Data = 1.2;


    
env_sizes = 64:32:320;
num_repeats = 10;
num_envs = length(env_sizes);

update_time_results = zeros(num_envs, 3);
update_freq_results = zeros(num_envs, 3);
update_time_std_results = zeros(num_envs, 3);
update_freq_std_results = zeros(num_envs, 3);

for e = 1:num_envs
    env_size = env_sizes(e);
    loop_stats = zeros(num_repeats, 3);
    
    disp("Env size: " + num2str(env_size));

    for k = 1:num_repeats
        disp("     Repeat: " + num2str(k));

        env = liveEnvironment(node, env_size, cell_size, origin, obstacle, scene);
        env.add_table_static_scene();

        start_sdf = env.getSDF();

        tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                        epsilon_dist, cell_size, env.dataset.origin_point3);

        pause(2);

        % Setup problem
        traj_publisher.goToConfig(start_conf);
        pause(3);

%         disp('Initialising sdfs');
        sdfs  = {};
        for i = 1:total_time_step+1
            sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                        cell_size, ...
                                                        env_size, ...
                                                        env_size, ...
                                                        env_size);
        end

        %% Setup problem and graph

        datasets = [];
        results = [];
        times = []; times = [times, 0];
        pub_msgs = [];
        confs = zeros(7, 100);

        panda_planner = pandaPlanner(start_sdf, problem_setup);
%         disp('Ready to simulate and execute');

        % Start the simulation
        t_update = 0;

        start_sim_pub.send(velocity_Msg); 

        i=1;
        t_step = 0;
        t_start = tic;

        env.updateMap();
        % First optimisation
        result = panda_planner.optimize(init_values);

        % Execute
        traj_publisher.publish(result, t_step);

        last_lock = 0;

        while t_update < total_time_sec
            t_update = toc(t_start);
            latest_msg = sub.LatestMessage;
            curr_conf = latest_msg.Position(3:end);
            curr_vel = latest_msg.Velocity(3:end);
            env.updateMap();

            times = [times, t_update];
            t_step = floor(t_update/delta_t);

            tracker.update(t_update, env.dataset.map);

            for p = last_lock:t_step
                panda_planner.update_confs(p, ...
                                            result.atVector(gtsam.symbol('x', p)), ...
                                            result.atVector(gtsam.symbol('v', p)));
            end
            last_lock = t_step + 1;

            for t = t_step:total_time_step
                forward_t = (t-t_step)*delta_t;
        %         forward_t = t_step*delta_t;

                field = tracker.predict_composite_sdf(forward_t);
        %         field = tracker.predict_field(t*delta_t);

                for z = 1:env_size
                    sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
                end

                panda_planner.update_sdf(t, sdfs{t+1});
            end

            result = panda_planner.optimize(init_values);

            % Execute
            if t_step < total_time_step -1
                traj_publisher.publish(result, t_step);
            end
            t_update = toc(t_start);
            i = i + 1;
        end
        times = [times, t_update];

        loop_times = times(3:end)-times(2:end-1);
        loop_stats(k,:) = [min(loop_times), median(loop_times), max(loop_times)];
    end % each repeat
    update_time_results(e,:) =  mean(loop_stats);
    update_freq_results(e,:) =  1./mean(loop_stats);
    update_time_std_results(e,:) =  std(loop_stats);
    update_freq_std_results(e,:) =  std(1./loop_stats);
end % Each env

%%

% save('/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/data/update_loop_timing', ...
%             'env_sizes', ...
%             'update_time_results', 'update_freq_results', ...
%             'update_time_std_results', 'update_freq_std_results')

% load('/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/data/update_loop_timing');


figure(1); hold on;
errorbar(env_sizes(1:end-3), update_freq_results(1:end-3,1), update_freq_std_results(1:end-3,1))
errorbar(env_sizes(1:end-3), update_freq_results(1:end-3,2), update_freq_std_results(1:end-3,2))
errorbar(env_sizes(1:end-3), update_freq_results(1:end-3,3), update_freq_std_results(1:end-3,3))
xlabel('Workspace Discretisation (Voxels per Side)');
ylabel('Update Loop Frequency (Hz)');
legend('Last Iteration', 'Median Iteration', 'First Iteration')
% ylim([0, 15]);
xticks(env_sizes);

figure(2); hold on;
errorbar(env_sizes(1:end-3), update_time_results(1:end-3,1), update_time_std_results(1:end-3,1))
errorbar(env_sizes(1:end-3), update_time_results(1:end-3,2), update_time_std_results(1:end-3,2))
errorbar(env_sizes(1:end-3), update_time_results(1:end-3,3), update_time_std_results(1:end-3,3))
xticks(env_sizes);
