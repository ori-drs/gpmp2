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
env_size = 64;
origin = [-1,-1,-1];

base_pos = [0, 0, 0.4];

% Setup ROS interactions
node = ros.Node('/matlab_node');

traj_publisher = kinovaTrajectoryPublisher(delta_t);
sub = ros.Subscriber(node,'/j2n6s300_driver/out/joint_state','sensor_msgs/JointState');
start_sim_pub = simulationPublisher(node, obstacle);

velocity_Msg = rosmessage('std_msgs/Float32');
% velocity_Msg.Data = 1.2;
velocity_Msg.Data = 1.2;

env = liveEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

start_sdf = env.getSDF();

tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);

% Setup problem
start_conf = setKinovaConf('ready');
end_conf = setPandaConf('left');

traj_publisher.goToConfig(start_conf);
pause(3);



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

% profile('on');
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

%     curr_sdf = env.getSDF(); % Only needed to update sdf
%     datasets = [datasets, env.dataset];

%     disp('Updating tracker');
    tracker.update(t_update, env.dataset.map);
%     disp(num2str(tracker.num_obs));

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
    end
    t_update = toc(t_start);


    i = i + 1;
%     profile off
end


% profile viewer
%% Plot state evolution
% load()
close all
figure('Renderer', 'painters', 'Position', [1000 1000 750 550]);
ax1 = axes('Position',[0.15 0.1 0.8 0.8]);
ax1.ActivePositionProperty = 'outerposition';

time_steps = 0:delta_t:total_time_sec;
conf_series = [];
for i=0:total_time_step
    conf = result.atVector(gtsam.symbol('v', i));
    conf_series = horzcat(conf_series, conf);
end

plot(time_steps, conf_series, 'LineWidth', 2);
xlabel('Time (s)', 'FontSize', 28);

% ylabel('Joint Position (rad)', 'FontSize', 28);
ylabel('Joint Velocity (rad/s)', 'FontSize', 28);


hleg = legend('joint1', 'joint2','joint3', 'joint4', ...
        'joint5','joint6','joint7', 'Location', 'north', ...
        'NumColumns',4, 'FontSize', 24);
% hleg.Position = [0.098    0.92    0.856    0.05];




save_dir ="/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/images/";
ylim([-4,4]);
set(gca,'FontSize',24);
set(gca,'XMinorTick','on','YMinorTick','on');

% saveas(gcf,save_dir + "prediction_state_results_x.png")
% saveas(gcf,save_dir + "exec_update_state_results_x.png")
saveas(gcf,save_dir + "prediction_state_results_v.png")
% saveas(gcf,save_dir + "exec_update_state_results_v.png")

%% Plot state evolution
close all
% figure('Renderer', 'painters', 'Position', [1000 1000 1100 550]);
figure('Renderer', 'painters', 'Position', [1000 1000 750 550]);
% ax1 = axes('Position',[0.1 0.1 0.85 0.8]);
ax1 = axes('Position',[0.15 0.1 0.8 0.8]);
ax1.ActivePositionProperty = 'outerposition';

time_steps = 0:delta_t:total_time_sec;
conf_series = [];
for i=0:total_time_step
    conf = result.atVector(gtsam.symbol('v', i));
    conf_series = horzcat(conf_series, conf);
end

plot(time_steps, conf_series, 'LineWidth', 2);
set(gca,'FontSize',24);

% hleg = legend('joint1', 'joint2','joint3', 'joint4', ...
%         'joint5','joint6','joint7', 'Location', 'Best', ...
%         'NumColumns',7, 'FontSize', 20);
% hleg.Position = [0.098    0.92    0.856    0.05];

hleg = legend('joint1', 'joint2','joint3', 'joint4', ...
        'joint5','joint6','joint7', 'Location', 'north', ...
        'NumColumns',4, 'FontSize', 24);


% ylabel('Joint Position (rad)', 'FontSize', 28);
ylabel('Joint Velocity (rad/s)', 'FontSize', 28);

xlabel('Time (s)', 'FontSize', 28);
save_dir ="/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/images/";
ylim([-4,4]);
% saveas(gcf,save_dir + "prediction_state_results_x.png")
% saveas(gcf,save_dir + "exec_update_state_results_x.png")
% saveas(gcf,save_dir + "prediction_state_results_v.png")
% saveas(gcf,save_dir + "exec_update_state_results_v.png")

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

