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

start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');

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

times = []; times = [times, 0];

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
result = panda_planner.optimize(init_values);

% Execute
traj_publisher.publish(result, t_step);

last_lock = 0;

time_to_predict = [];
while t_update < total_time_sec
    % Get latest conf and sdf to update
    t_update = toc(t_start);

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

    result = panda_planner.optimize(init_values);

    % Execute
    disp('Publishing trajectory');
    if t_step < total_time_step -1
        traj_publisher.publish(result, t_step);
    end
    t_update = toc(t_start);


    i = i + 1;
end


%% Analyse the prediction frequency

% We want, the min, max, average + initial cosat
row_names = {'Prediction Time (ms)'; 'Frequency (Hz)'};
min_time = min(time_to_predict) * 1000;
median_time = median(time_to_predict) * 1000;
mean_time = mean(time_to_predict) * 1000;
max_time = max(time_to_predict) * 1000;
time_std_dev = std(time_to_predict) * 1000;

Min = [min(time_to_predict) * 1000; 1/max(time_to_predict)];
Median = [median(time_to_predict) * 1000; 1/median(time_to_predict)];
Mean = [mean(time_to_predict) * 1000; 1/mean(time_to_predict)];
Max = [max(time_to_predict) * 1000; 1/min(time_to_predict) ];
Std = [std(time_to_predict) * 1000; std(1./time_to_predict)];

T = table(row_names, Min, Median, Mean, Max, Std)

% Save results
writetable(T,"/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/data/sdf_prediction_rate_96env")
save('/home/mark/installs/gpmp2/mark_gpmp2/paper/gazebo_experiments/data/sdf_prediction_time96env','time_to_predict')
