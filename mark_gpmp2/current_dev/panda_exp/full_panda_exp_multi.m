clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

use_gazebo = false;

plot_graphs = false;
total_time_sec = 3.0;
delta_t = 0.1;
interp_multiplier = 5;
cost_sigma = 0.03;
epsilon_dist = 0.25;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
env_size = 75;
origin = [-1,-1,-1];
base_pos = [0, 0, 0.5];

% Setup ROS interactions
% rosinit;
node = ros.Node('/matlab_node');

traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
pub = ros.Publisher(node,'/start_simulation','std_msgs/String');
strMsg = rosmessage('std_msgs/String');

% Gazebo usage
if use_gazebo
    env = liveEnvironment(node, env_size, cell_size, origin);
    env.add_table_static_scene();
    start_sdf = env.getSDF();
else
    env = loadPredefinedMovingEnvironment('PandaEnvOnePillar', 150, 0.02, [-1,-1,-1]);
    start_sdf = dataset.sdf;
end


tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);

% Setup problem

if use_gazebo
    current_joint_msg = sub.LatestMessage;
    start_conf = current_joint_msg.Position(3:end);
else
    start_conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
end


end_conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
% end_conf =  [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';
% end_conf =  [1.3877,-0.4045,-0.0316,-1.0321,-2.7601,2.4433,0.5000]';
    
total_time_step = round(total_time_sec/delta_t);
problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos);             

init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);


disp('Initialising sdfs');
sdfs  = cell(1,total_time_step+1);
for i = 1:total_time_step+1
    sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                cell_size, ...
                                                env_size, ...
                                                env_size, ...
                                                env_size);
end

panda_planner = pandaPlanner(start_sdf, problem_setup);

datasets = [];
results = [];
times = []; times = [times, 0];
pub_msgs = [];
confs = zeros(7, 100);

%% Setup problem and graph
% pause(4);

disp('Ready to simulate and execute');

% Start the simulation
t_update = 0;
i=1;
t_step = 0;

if use_gazebo
    pub.send(strMsg); 
end

t_start = tic;

if use_gazebo
    env.updateMap();
else
    curr_t = toc(t_start);
    env.updateMap(curr_t)
end

% First optimisation
disp('Optimising');
result = panda_planner.optimize(init_values);

% Execute
disp('Publishing trajectory');
if use_gazebo
    traj_publisher.publish(result, t_step);
else
    
end

datasets = [datasets, env.dataset];
results = [results, result];
last_lock = 0;

% profile on
while t_update < total_time_sec
    % Get latest conf and sdf to update
    disp('Updating map and conf');
    t_update = toc(t_start);
%     latest_msg = sub.LatestMessage;
%     curr_conf = latest_msg.Position(3:end);
%     curr_vel = latest_msg.Velocity(3:end);
    env.updateMap();
%     confs(:,i) = curr_conf;
    
    disp(t_update);
%     times = [times, t_update];
    t_step = floor(t_update/delta_t);
        
%     curr_sdf = env.getSDF(); % Only needed to update sdf
%     datasets = [datasets, env.dataset];
    
    disp('Updating tracker');
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
plot_graphs = true;

if plot_graphs

    frame = 10;
    traj = results(frame);

    lab_axis_lims = [-1 2 -1 2 -1 2];

    [X, Y, Z] = getEnvironmentMesh(datasets(1));
    figure(2); hold on; cla;
    set(gcf,'Position',[1350 500 1200 1400]);
    axis(lab_axis_lims); grid on; view(3);
    xlabel('x'); ylabel('y'); zlabel('z');


    % h1 = plot3DEnvironment(datasets(frame), X, Y, Z);
    h1 = plot3DEnvironment(datasets(frame).map, X, Y, Z);


    t_s = floor(times(frame)/delta_t);

    for i = t_s:problem_setup.total_time_step
    %     cla;
        key_pos = gtsam.symbol('x', i);
        conf = traj.atVector(key_pos);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
            key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
            problem_setup.epsilon_dist);

    %     h1 = plot3DEnvironment(datasets(i+1).map, X, Y, Z);

        static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);
% 
%         if any(obs_factor.spheresInCollision(conf))
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%         else
%             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%         end

        pause(0);
    end

end