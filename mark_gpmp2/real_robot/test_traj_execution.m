clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

scene = "big_room"; % "bookshelf" 
obstacle = "ar_box";

plot_graphs = false;
total_time_sec = 5;
delta_t = 0.2;
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

panda_planner = pandaPlanner(start_sdf, problem_setup);
t_update = 0;
t_step = 0;
t_start = tic;

env.updateMap();

% First optimisation
result = panda_planner.optimize(init_values);
traj_publisher.publish(result, t_step, 0);

lag_steps = 3;


pause(1);

t_update = toc(t_start);
t_step = floor((t_update)/delta_t);
traj_publisher.publish(result, t_step, lag_steps);

pause(1);

t_update = toc(t_start);
t_step = floor((t_update)/delta_t);
traj_publisher.publish(result, t_step, lag_steps);

pause(1);

t_update = toc(t_start);
t_step = floor((t_update)/delta_t);
traj_publisher.publish(result, t_step, lag_steps);

% 
% t0 = tic;
% latest_msg = joint_sub.LatestMessage;
% curr_conf = latest_msg.Position;
% curr_vel = latest_msg.Velocity;
% traj_publisher.publish(result,0);
% t1 = toc(t0);
