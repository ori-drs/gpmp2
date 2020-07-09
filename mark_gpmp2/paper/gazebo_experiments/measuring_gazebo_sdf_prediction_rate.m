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

scene = "tables"; % "bookshelf" 
obstacle = "hsrb";

cell_size = 0.04;
env_size = 96;
origin = [-1,-1,-1];

% Setup ROS interactions
% rosinit;
node = ros.Node('/matlab_node');

traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');

% pub = ros.Publisher(node,'/start_simulation','std_msgs/String');
% person_pub = ros.Publisher(node,'/start_moving_person','std_msgs/String');
hsrb_pub = ros.Publisher(node,'/start_moving_hsrb','std_msgs/String');
% panda_cylinder_pub = ros.Publisher(node,'/start_moving_panda_cylinder','std_msgs/String');

strMsg = rosmessage('std_msgs/String');

env = liveEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

start_sdf = env.getSDF();

tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);

% Setup problem
base_pos = [0, 0, 0.4];

current_joint_msg = sub.LatestMessage;
curr_conf = current_joint_msg.Position(3:end);
ready_conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
% end_conf = start_conf;
% side_conf =  [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';
% end_conf =  [1.3877,-0.4045,-0.0316,-1.0321,-2.7601,2.4433,0.5000]';

left_forward_conf = [0.20, 0.63, 0.24, -2.01, -0.28, 2.61, 1.42]';
right_forward_conf = [-0.65, 0.65, 0.18, -1.94, -0.21, 2.58, 0.46]';
in_shelf_conf = [-2.40, -1.44, 1.11, -1.76, 2.41, 1.78, 2.80]';
right_conf = [-0.32, -1.76, -1.42, -2.69, -1.52, 1.33, 0]';
right_ready_conf = [-1.57, -0.785, 0, -2.356, 0, 1.57, 0.785]';
forward_conf = [0, 0.94, -0.07, -1.27, 0.07, 2.21, 0.70]';

% Shelf confs
top_shelf_conf = [-1.32, 1.42, 1.85, -1.54, -2.61, 2.70, 1.85]';
behind_conf = [-3.14, -0.785, 0, -2.356, 0, 1.57, 0.785]';
left_conf = [1.90, 0.64, 0.01, -1.72, -0.01, 2.36, 1.14]';
left_in_shelf_conf = [-0.51, 1.26, 1.80, -1.23, -2.80, 2.20, 2.0]';

start_conf = curr_conf;
end_conf = forward_conf;
% end_conf = right_ready_conf;

total_time_step = round(total_time_sec/delta_t);
problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
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
% pause(4);

loop_times = zeros(10);

% for r = 1:10
datasets = [];
results = [];
times = []; times = [times, 0];
pub_msgs = [];
confs = zeros(7, 100);

panda_planner = pandaPlanner(start_sdf, problem_setup);

disp('Ready to simulate and execute');
% Start the simulation
t_update = 0;
hsrb_pub.send(strMsg); 

i=1;
t_step = 0;
t_start = tic;

env.updateMap();
% First optimisation
% disp('Optimising');
result = panda_planner.optimize(init_values);

% Execute
% disp('Publishing trajectory');
traj_publisher.publish(result, t_step);

% results = [results, result];
last_lock = 0;

time_to_predict = [];
while t_update < total_time_sec
    % Get latest conf and sdf to update
%     disp('Updating map and conf');
    t_update = toc(t_start);
    latest_msg = sub.LatestMessage;
    curr_conf = latest_msg.Position(3:end);
    curr_vel = latest_msg.Velocity(3:end);
    env.updateMap();
%     confs(:,i) = curr_conf;

    disp(t_update);
    times = [times, t_update];
    t_step = floor(t_update/delta_t);

    curr_sdf = env.getSDF(); % Only needed to update sdf
%     datasets = [datasets, env.dataset];

%     disp('Updating tracker');
    tracker.update(t_update, env.dataset.map);
    disp(num2str(tracker.num_obs));

    for p = last_lock:t_step
        panda_planner.update_confs(p, ...
                                    result.atVector(gtsam.symbol('x', p)), ...
                                    result.atVector(gtsam.symbol('v', p)));
    end
    last_lock = t_step + 1;

%     disp('Updating factor graph');
    for t = t_step:total_time_step
        forward_t = (t-t_step)*delta_t;
        t_predict_ref = toc(t_start);
        field = tracker.predict_composite_sdf(forward_t);
        t_predict = toc(t_start) - t_predict_ref;
        time_to_predict(end+1) = t_predict;
        
        for z = 1:env_size
            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
        end

        panda_planner.update_sdf(t, sdfs{t+1});
    end

%     disp('Reoptimising');
    result = panda_planner.optimize(init_values);
%     result = panda_planner.optimize(result);
%     results = [results, result];

    % Execute
    disp('Publishing trajectory');
    if t_step < total_time_step -1
        traj_publisher.publish(result, t_step);
    end
    t_update = toc(t_start);


    i = i + 1;
end
%     
%     pause(5);
%     loop_times(r+1) = times(3)-times((2));
% end
