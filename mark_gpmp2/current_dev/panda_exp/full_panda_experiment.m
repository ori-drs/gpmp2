clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%%
total_time_sec = 3.0;
delta_t = 0.1;
interp_multiplier = 20;
cost_sigma = 0.05;
epsilon_dist = 0.2;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
env_size = 75;
origin = [-1,-1,-1];

%% Setup ROS interactions
% rosinit;
node = ros.Node('/matlab_node5');

traj_publisher = trajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/joint_states','sensor_msgs/JointState');
pub = ros.Publisher(node,'/start_simulation','std_msgs/String');
strMsg = rosmessage('std_msgs/String');

env = liveEnvironment(node, env_size, cell_size, origin);
start_sdf = env.getSDF();

tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);

%% Setup problem
% arm = myGenerateArm('Panda', gtsam.Pose3(gtsam.Rot3(eye(3)), gtsam.Point3([0,0,1.5]')));
% arm_model = arm.fk_model();
base_pos = [0, 0, 0.5];

current_joint_msg = sub.LatestMessage;
start_conf = current_joint_msg.Position(3:end);
% end_conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
% end_conf = start_conf;
end_conf =  [1.57,           0.185,   0,       -1.70,   0,        3.14,   0]';

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

panda_planner = pandaPlanner(start_sdf, problem_setup);
disp('Ready to simulate and execute');
% Start the simulation
t_update = 0;
pub.send(strMsg); 

i=1;
t_step = 0;
t_start = tic;

env.updateMap();
% First optimisation
disp('Optimising');
result = panda_planner.optimize(init_values);

% Execute
disp('Publishing trajectory');
traj_publisher.publish(result, t_step);

% profile on
datasets = []; datasets = [datasets, env.dataset];
results = []; results = [results, result];
times = []; times = [times, 0];

while t_update < total_time_sec
    % Get latest conf and sdf to update
    disp('Updating map and conf');
    latest_msg = sub.LatestMessage;
    curr_conf = latest_msg.Position(3:end);
    curr_vel = latest_msg.Velocity(3:end);
    env.updateMap();
    
    t_update = toc(t_start);
    disp(t_update);
    times = [times, t_update];
    t_step = floor(t_update/delta_t);
        
    curr_sdf = env.getSDF();
    datasets = [datasets, env.dataset];
    
    disp('Updating tracker');
    tracker.update(t_update, env.dataset.map);
    
    disp('Calculating SDF');
%     field = tracker.predict_sdf(t_update);

    disp('Updating factor graph');
    for t = t_step:total_time_step
%         field = tracker.predict_composite_sdf(t);
        field = tracker.predict_field(t*delta_t);

        for z = 1:size(field, 3)
            sdfs{t}.initFieldData(z-1, field(:,:,z)');
        end

        panda_planner.update_confs(t_step, curr_conf, curr_vel);
        panda_planner.update_sdf(t, sdfs{t});
    end

    disp('Reoptimising');
    result = panda_planner.optimize(init_values);
    results = [results, result];
    
    % Execute
    disp('Publishing trajectory');
    traj_publisher.publish(result, t_step);
    t_update = toc(t_start);
    i = i + 1;

end

% profile viewer

%% 

% lab_axis_lims = [-1 2 -1 2 -1 2];
% 
% [X, Y, Z] = getEnvironmentMesh(datasets(1));
% figure(2); hold on; cla;
% set(gcf,'Position',[1350 500 1200 1400]);
% axis(lab_axis_lims); grid on; view(3);
% xlabel('x'); ylabel('y'); zlabel('z');
% 
% frame = 5;
% traj = results(frame);
% 
% h1 = plot3DEnvironment(datasets(frame), X, Y, Z);
% 
% for i = 0:problem_setup.total_time_step
%     key_pos = gtsam.symbol('x', i);
%     conf = traj.atVector(key_pos);
% 
% %     gpmp2.plotRobotModel(problem_setup.arm, conf);
%     gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
% %     pause(0.2);
% end
% 

frame = 4;
traj = results(frame);


lab_axis_lims = [-1 2 -1 2 -1 2];

[X, Y, Z] = getEnvironmentMesh(datasets(1));
figure(2); hold on; cla;
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');


h1 = plot3DEnvironment(datasets(frame), X, Y, Z);

for i = 0:problem_setup.total_time_step
    key_pos = gtsam.symbol('x', i);
    conf = traj.atVector(key_pos);
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
        problem_setup.epsilon_dist);


    if any(obs_factor.spheresInCollision(conf))
%         disp('COLLISION');
        static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
    else
        static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
    end

    pause(0);
end