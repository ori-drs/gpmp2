clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%%
total_time_sec = 3.0;
delta_t = 0.1;
epsilon_dist = 0.2;    

cell_size = 0.04;
env_size = 75;
origin = [-1,-1,-1];

%% Setup ROS interactions
% rosinit;
node = ros.Node('/matlab_node');

traj_publisher = trajectoryPublisher(delta_t);
pub = ros.Publisher(node,'/start_simulation','std_msgs/String');
strMsg = rosmessage('std_msgs/String');

env = liveEnvironment(node, env_size, cell_size, origin);
start_sdf = env.getSDF();

tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                                epsilon_dist, cell_size, env.dataset.origin_point3);
                            
pause(2);


%% Setup problem and graph

disp('Ready to simulate and execute');
% Start the simulation
t_update = 0;
pub.send(strMsg); 

t_start = tic;

env.updateMap();

datasets = []; datasets = [datasets, env.dataset];
times = []; times = [times, 0];

predicted_maps = {};
while t_update < total_time_sec
    % Get latest conf and sdf to update
    disp('Updating map and conf');
    env.updateMap();
    

    t_update = toc(t_start);
    disp(t_update);
    times = [times, t_update];
%     t_step = floor(t_update/delta_t);
    
    disp('Updating tracker');
    tracker.update(t_update, env.dataset.map);
    
    curr_sdf = env.getSDF();
    datasets = [datasets, env.dataset];
    predicted_maps{end+1} = tracker.predict_sdf(t_update);

end






%%

lab_axis_lims = [-1 2 -1 2 -1 2];

[X, Y, Z] = getEnvironmentMesh(datasets(1));
figure(2); hold on; cla;
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');



for i = 1:length(predicted_maps)
% for i = 1:length(datasets)
%     key_pos = gtsam.symbol('x', i);
%     conf = traj.atVector(key_pos);
%     obs_factor = gpmp2.ObstacleSDFFactorArm(...
%         key_pos, problem_setup.arm, datasets(frame).sdf, problem_setup.cost_sigma, ...
%         problem_setup.epsilon_dist);
    cla;
    h1 = plot3DEnvironment(predicted_maps{i}, X, Y, Z);
%     h1 = plot3DEnvironment(datasets(i), X, Y, Z);

%     if any(obs_factor.spheresInCollision(conf))
%         disp('COLLISION');
%         static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
%     else
%         static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
%     end

    pause(0.2);
end