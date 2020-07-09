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

% Setup ROS interactions
node = ros.Node('/matlab_node');

traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
start_sim_pub = simulationPublisher(node, obstacle);

velocity_Msg = rosmessage('std_msgs/Float32');
velocity_Msg.Data = 1.4;

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

%     panda_planner.update_confs(t_step, curr_conf, curr_vel);

    for p = last_lock:t_step
        panda_planner.update_confs(p, ...
                                    result.atVector(gtsam.symbol('x', p)), ...
                                    result.atVector(gtsam.symbol('v', p)));
    end
    last_lock = t_step + 1;

%     disp('Updating factor graph');
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

%     disp('Reoptimising');
    result = panda_planner.optimize(init_values);
%     result = panda_planner.optimize(result);
%     results = [results, result];

    % Execute
    disp('Publishing trajectory');
    if t_step < total_time_step -1
        traj_publisher.publish(result, t_step);
%         for k = 1:size(traj_publisher.rGoalMsg.Trajectory.Points,1)
%             pub_msgs(((t_step + k-1)*7)+(1:7),i) = traj_publisher.rGoalMsg.Trajectory.Points(k).Positions;
%         end
    end
    t_update = toc(t_start);


    i = i + 1;
%     profile off
end


% profile viewer

%% 

disp_traj = displayTrajectory(node, 0.01);
disp_traj.publish(results(1), 1);

plot_graphs = true;

if plot_graphs

    frame = 8;
    traj = results(frame);

    lab_axis_lims = [-1 2 -1 2 -1 2];

    [X, Y, Z] = getEnvironmentMesh(datasets(1));
    figure(2); hold on; cla;
%     set(gcf,'Position',[1350 500 1200 1400]);
%     axis(lab_axis_lims); 
    grid on; 
    view(3);
    xlabel('x'); ylabel('y'); zlabel('z');


    % h1 = plot3DEnvironment(datasets(frame), X, Y, Z);
    h1 = plot3DEnvironment(datasets(frame).map, X, Y, Z);


    t_s = floor(times(frame)/delta_t);
    step_size = 3;
    for i = t_s:step_size:problem_setup.total_time_step
%     for i = problem_setup.total_time_step:step_size:problem_setup.total_time_step
    %     cla;
        key_pos = gtsam.symbol('x', i);
        conf = traj.atVector(key_pos);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
            key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
            problem_setup.epsilon_dist);

        h1 = plot3DEnvironment(datasets(frame).map, X, Y, Z);

        static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);

%         if any(obs_factor.spheresInCollision(conf))
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%         else
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%         end

        pause(0.1);
    end

end


%% Plot each frame
plot_graphs = true;

if plot_graphs

    [X, Y, Z] = getEnvironmentMesh(datasets(1));
    figure(2); hold on; cla;
    lab_axis_lims = [-1 2 -1 2 -1 2];
    %     set(gcf,'Position',[1350 500 1200 1400]);
    %     axis(lab_axis_lims); 
    grid on; 
    view(3);
    xlabel('x'); ylabel('y'); zlabel('z');
        
    for frame = 1:length(results)
        cla;
        traj = results(frame);

        h1 = plot3DEnvironment(datasets(frame).map, X, Y, Z);


        t_s = floor(times(frame)/delta_t);
        
        key_pos = gtsam.symbol('x', t_s);
        conf = traj.atVector(key_pos);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
            key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
            problem_setup.epsilon_dist);

        h1 = plot3DEnvironment(datasets(frame).map, X, Y, Z);

        static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);

%         if any(obs_factor.spheresInCollision(conf))
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%         else
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%         end

        pause(0.5);
    end
end


%% Plot the end effector trajectory and how it progresses

lab_axis_lims = [-1 2 -1 2 -1 2];

[X, Y, Z] = getEnvironmentMesh(datasets(1));
figure(2); hold on; cla;
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');

h1 = plot3DEnvironment(datasets(2).map, X, Y, Z);


plotEndEffectorTrajectory(problem_setup.arm.fk_model(), results(1), 'g');
hold on;
plotEndEffectorTrajectory(problem_setup.arm.fk_model(), results(10), 'm');



% 
% 
% frame = 1;
% load('/home/mark/installs/gpmp2/mark_gpmp2/data/live_panda_actual_maps.mat');
% 
% lab_axis_lims = [-1 2 -1 2 -1 2];
% 
% [X, Y, Z] = getEnvironmentMesh(datasets(1));
% figure(2); hold on; cla;
% set(gcf,'Position',[1350 500 1200 1400]);
% axis(lab_axis_lims); grid on; view(3);
% xlabel('x'); ylabel('y'); zlabel('z');
% 
% t_ind = 1;
% t_s = 0;
% 
% for i = 0:problem_setup.total_time_step
%     
%     if i >= ceil(times(t_ind+1)/delta_t)
%         t_ind = t_ind + 1;
%     end
%     
%     traj = results(t_ind);
% 
%     cla;
%     key_pos = gtsam.symbol('x', i);
%     conf = traj.atVector(key_pos);
%     obs_factor = gpmp2.ObstacleSDFFactorArm(...
%         key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
%         problem_setup.epsilon_dist);
% 
%     h1 = plot3DEnvironment(even_actual_maps{i+1}, X, Y, Z);
% 
%     static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);
% 
%     pause(0.2);
% end


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