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
base_pos = [0, 0, 0.4];

cell_size = 0.04;
env_size = 96;
origin = [-1,-1,-1];


total_time_step = round(total_time_sec/delta_t);


% Setup ROS interactions
node = ros.Node('/matlab_node');

% Set up ROS communications
traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
start_sim_pub = simulationPublisher(node, obstacle);

velocity_Msg = rosmessage('std_msgs/Float32');
velocity_Msg.Data = 1.4;

% Set up environment monitor -- this resets itself anyway. No need to be in
% loop
env = liveEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

% Pre-allocate memory for the SDFs
disp('Initialising sdfs');
sdfs  = {};
for i = 1:total_time_step+1
    sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                cell_size, ...
                                                env_size, ...
                                                env_size, ...
                                                env_size);
end
%%
% Reset experiment
start_conf = setConf('right_ready');
end_conf = setConf('forward_conf');
init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);
problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos); 
                                 
traj_publisher.goToConfig(start_conf);
tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);                            
pause(3);

% Get the starting environment
env.resetMap();
start_sdf = env.getSDF();


%% Setup problem and graph
% pause(4);

datasets = [];
results = [];
times = []; times = [times, 0];
pub_msgs = [];
confs = zeros(7, 100);

panda_planner = pandaPlanner(start_sdf, problem_setup);

disp('Ready to simulate and execute');
% Start the simulation
t_update = 0;

start_sim_pub.send(velocity_Msg); 

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

% datasets = [datasets, env.dataset];
% results = [results, result];
last_lock = 0;

% profile on

while t_update < total_time_sec
    t_update = toc(t_start);

    % Make observation
    env.updateMap();

    times = [times, t_update];
    t_step = floor(t_update/delta_t);

    % Update the tracker
    tracker.update(t_update, env.dataset.map);
    disp(num2str(tracker.num_obs));

    % Fix confs
    for p = last_lock:t_step
        panda_planner.update_confs(p, ...
                                    result.atVector(gtsam.symbol('x', p)), ...
                                    result.atVector(gtsam.symbol('v', p)));
    end
    last_lock = t_step + 1;

    % Update the obstacle factors
    for t = t_step:total_time_step
        forward_t = (t-t_step)*delta_t;
%         forward_t = t_step*delta_t;
        field = tracker.predict_composite_sdf(forward_t);

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





%% Functions

function conf = setConf(conf_name)

    switch conf_name
        case 'ready'
            conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
        case 'left_forward'
            conf = [0.20, 0.63, 0.24, -2.01, -0.28, 2.61, 1.42]';
        case 'right_forward'
            conf = [-0.65, 0.65, 0.18, -1.94, -0.21, 2.58, 0.46]';
        case 'in_shelf'
            conf = [-2.40, -1.44, 1.11, -1.76, 2.41, 1.78, 2.80]';
        case 'right_ready'
            conf = [-1.57, -0.785, 0, -2.356, 0, 1.57, 0.785]';
        case 'forward'
            conf = [0, 0.94, -0.07, -1.27, 0.07, 2.21, 0.70]';
        case 'top_shelf'
            conf = [-1.32, 1.42, 1.85, -1.54, -2.61, 2.70, 1.85]';
        case 'behind'
            conf = [-3.14, -0.785, 0, -2.356, 0, 1.57, 0.785]';
        case 'left'
            conf = [1.90, 0.64, 0.01, -1.72, -0.01, 2.36, 1.14]';
        case 'left_in_shelf'
            conf = [-0.51, 1.26, 1.80, -1.23, -2.80, 2.20, 2.0]';
    end
end

function pub = simulationPublisher(node, obstacle)

    switch obstacle    
        case 'person'
            pub = ros.Publisher(node,'/start_moving_person','std_msgs/String');    
        case 'cylinder'
            pub = ros.Publisher(node,'/start_moving_panda_cylinder','std_msgs/String');    
        case 'hsrb'
            pub = ros.Publisher(node,'/start_moving_hsrb','std_msgs/Float32');
    end

end