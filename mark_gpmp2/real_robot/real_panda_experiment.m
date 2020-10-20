clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

scene = "big_room"; % "bookshelf" 
obstacle = "Tag0";

lag_steps = 2;

plot_graphs = false;
total_time_sec = 5;
delta_t = 0.1;
interp_multiplier = 1;
cost_sigma = 0.05;
epsilon_dist = 0.2;    
limit_v = true;
limit_x = false;

cell_size = 0.04;
env_size = 64;
origin = [-1,-1,-1];

base_pos = [0, 0, 0];

% Setup ROS interactions
node = ros.Node('/matlab_node');

traj_publisher = realPandaTrajectoryPublisher(delta_t);
joint_sub = ros.Subscriber(node,'/franka_state_controller/joint_states','sensor_msgs/JointState');

env = realEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

start_sdf = env.getSDF();

tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);

% Setup problem
start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');
traj_publisher.goToConfig(start_conf);
pause(3);



total_time_step = round(total_time_sec/delta_t);
problem_setup = pandaProblemSetup(start_conf, end_conf, ...
                                    total_time_sec, delta_t, ...
                                    cost_sigma, epsilon_dist, interp_multiplier, ...
                                    limit_x, limit_v, base_pos);             

init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);


disp('Initialising sdfs');
sdfs  = {};
for i = 1:total_time_step+1
    sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                cell_size, ...
                                                env_size, ...
                                                env_size, ...
                                                env_size);
end



%% Setup problem and graph

results = [];
panda_planner = pandaPlanner(start_sdf, problem_setup);

disp('Ready to simulate and execute');
% Start the simulation
t_update = 0;

i=1;
t_step = 0;
t_start = tic;
rostime_start = rostime('now');

env.updateMap();

% First optimisation
result = panda_planner.optimize(init_values);
results = [results, result];

% Execute
% traj_publisher.publish(result, t_step, start_conf, zeros(7,1));
traj_publisher.publish(result, t_step, 0);
rostime_start = rostime('now');


last_lock = 0;

while t_update < total_time_sec
    %Get latest conf and sdf to update
    t_update = toc(t_start);
%     t0 = clock;
    latest_msg = joint_sub.LatestMessage;
    curr_conf = latest_msg.Position;
    curr_vel = latest_msg.Velocity;
    env.updateMap();
%     msg_rostime = latest_msg.Header.Stamp - rostime_start;
%     msg_rostime = max(msg_rostime.seconds, 0);
    disp(t_update);

    t_step = floor(t_update/delta_t);
%     t_step = floor(msg_rostime/delta_t);
    disp(t_step);
    tracker.update(t_update, env.dataset.map);
    
%     panda_planner.update_confs(t_step, curr_conf, curr_vel);
    for p = last_lock:t_step
        panda_planner.update_confs(p, ...
                                    result.atVector(gtsam.symbol('x', p)), ...
                                    result.atVector(gtsam.symbol('v', p)));
    end
    last_lock = t_step + 1;

    disp('Updating factor graph');
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

    disp('Reoptimising');
    result = panda_planner.optimize(init_values);

%     result = panda_planner.optimize(result);
    results = [results, result];

    % Execute
    disp('Publishing trajectory');
    t_update = toc(t_start);
%     t_update = rostime('now') - rostime_start;
%     t_update = t_update.seconds;

    t_step = floor(t_update/delta_t);
    if t_step+lag_steps < total_time_step -1
%         ms = round(etime(clock,t0) * 1000);
        latest_msg = joint_sub.LatestMessage;
        curr_conf = latest_msg.Position;
        curr_vel = latest_msg.Velocity;
%         traj_publisher.publish(result, t_step, curr_conf, curr_vel);
        traj_publisher.publish(result, t_step, lag_steps);
%         disp([ curr_conf, result.atVector(gtsam.symbol('x', t_step))])
        disp(curr_conf-result.atVector(gtsam.symbol('x', t_step)))
    end
    t_update = toc(t_start);


    i = i + 1;
end

%%
% f = figure(1);
% results_vis(results, delta_t, 10, f)

figure(4);
hold on;
h2 = subplot(1, 1, 1);
plotStateEvolution(results(1), delta_t, total_time_sec, total_time_step, 'v')
title('Iteration:' + string(k+1))
hold off
        