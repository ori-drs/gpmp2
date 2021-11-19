clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
plot_graphs = false;
total_time_sec = 5;
delta_t = 0.02;
interp_multiplier = 10;
cost_sigma = 0.05;
epsilon_dist = 0.1;    
limit_v = true;
limit_x = false;
cell_size = 0.02;
env_size = 128;
origin = [-1.28,-1.28,-1.28];
base_pos = [0, 0, 0];
total_time_step = round(total_time_sec/delta_t);


%% Environment

% Obstacle def
starting_pos1 = [1.0, -0.35, 0.35];
obs_size = [1.0, 0.3, 1];

% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object(starting_pos1, obs_size);
% env.add_object(starting_pos1, obs_size);
env.updateMap();

env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object([1.0, -0.35, 0], [0.9, 0.3, 0.8]);
env.add_object([0.4, -0.35, -0.1], [0.4, 0.3, 0.6]);
env.add_object([0.0, -0.35, 1.0], [3.0, 0.3, 0.6]);

env.add_object([0, -0.35, -0.15], [3.0, 0.3, 0.54]);

env.add_object([1.2, -0.35, 0.6], [0.2, 0.3, 0.6]);

env.add_object([0.0, -0.35, 0.6], [0.6, 0.3, 0.2]);

env.add_object([-1.0, -0.35, 0.35], [1.4, 0.3, 1]);

% Add floor 
env.add_object([0, 0, 0], [3.0, 3.0, 0.05]);


% dataset = env.queryEnv();
% sdf = dataset.sdf;
use_signed = true;
if use_signed
    sdf = env.getSDF();
else
    sdf = env.getUSDF();
end

%% Task

% Setup problem

% Example 1: sdf = 0 col, usdf = 1
% start_conf = [1.9, 0.64, 0.01, -1.72, -0.01, 2.36, 1.14]';
% end_conf =  [-1.9, 1.5, 0.1, -0.2, 1.57, 3, 1.14]';

% Example 2: sdf = 2 col. usdf = 0
% start_conf =  [1.3, 0.44, 0.11, -1.52, 0.1, 1.36, 0.6]';
% end_conf =  [-1.8, 1.5, 0.5, -0.2, 1.57, 2, 1.0]';

start_conf =  [1.5700,   -0.7850 ,        0,   -2.3560,         0,    1.5700,    0.7850]';
end_conf =  [-1.8, 1.5, 0.5, -0.2, 1.57, 2, 1.0]';


problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                     cost_sigma, epsilon_dist, interp_multiplier, ...
                                     limit_x, limit_v, base_pos);             


%% Planning
init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

panda_planner = pandaGraphMaintainer(sdf, problem_setup);

[result, error, iterations] = panda_planner.optimize(init_values);


%% Analysis

% How many collisions?
collision_timeline = zeros(total_time_step + 1);
for t = 0:total_time_step
    key_pos = gtsam.symbol('x', t);
    conf = result.atVector(key_pos);
    collision_timeline(t+1) = panda_planner.collisionCheck(conf);
    num_collisions = sum(collision_timeline, 'all');
end

disp("Error: " + num2str(error) + sprintf('\t') + "Iterations: " + num2str(iterations) +  sprintf('\t')  + "Collisions: " + num2str(num_collisions));
% How many iterations?
% Total error
% Time taken to plan

results_log = zeros(40000, 6); % [index, signed(1)/unsigned(0) error, collisions, iterations]
results_log(1, :) = double([1, 1, 1, error, double(num_collisions), double(iterations)]);


%% Visualisations

% Plot simulation

dataset = env.queryEnv();
[X, Y, Z] = getEnvironmentMesh(dataset);
figure(1); hold on; cla;
lab_axis_lims = [-1.5 1.5 -1.5 1.5 -0.5 1];
set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); 
grid on; 
view(3);
xlabel('x'); ylabel('y'); zlabel('z');



obs_factor = gpmp2.ObstacleSDFFactorArm(...
    key_pos, problem_setup.arm, sdf, problem_setup.cost_sigma, ...
    problem_setup.epsilon_dist);

for t = 0:total_time_step
cla;
h1 = plot3DEnvironment(dataset.map, X, Y, Z);

key_pos = gtsam.symbol('x', t);
conf = result.atVector(key_pos);

h1 = plot3DEnvironment(dataset, X, Y, Z);

static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);

if(any(obs_factor.spheresInCollision(conf)))
    disp("Collision");
end

pause(0.05);
end



%% 
% Setup ROS interactions
node = ros.Node('/matlab_node');
isFake = false;

disp_traj = displayTrajectory(node, 1);
disp_traj.publish(result, 0);

traj_publisher = trajectoryPublisher(delta_t, isFake);
pause(2);

traj_publisher.publish(result, 0);
%%


val = env.getUSDF();
usdf_field = env.dataset.field(:, 50, :);
usdf_field = squeeze(usdf_field);
val = env.getSDF();
sdf_field = env.dataset.field(:, 50, :);
sdf_field = squeeze(sdf_field);


        
figure(2);
% get X-Y coordinates
grid_rows = size(env.dataset.field, 1);
grid_cols = size(env.dataset.field, 2);
grid_corner_x = origin(1) + (grid_cols-1)*cell_size;
grid_corner_y = origin(2) + (grid_rows-1)*cell_size;
grid_X = origin(1) : cell_size : grid_corner_x;
grid_Y = origin(2) : cell_size : grid_corner_y;

a = env.dataset.field(:,100,:);
a = squeeze(a);
h = imagesc(grid_X, grid_Y, a);

h = imagesc(grid_X', grid_Y', fliplr(flipud(a')));
set(gca, 'YTickLabel', [1.2,1.0,0.8,0.6,0.4, 0.2, 0.0])
set(gca, 'XTickLabel', [-1,-0.5,0,0.5,1])
ylim([-1.2, 0.3]);
colormap(jet(128))

% colormap(gray(64))
% caxis([min(min(field)), epsilon_dist]);

set(gca,'YDir','reverse')
set(gca,'FontSize',18)

axis equal
axis([origin_x-cell_size/2, grid_corner_x+cell_size/2, ...
    origin_y-cell_size/2, grid_corner_y+cell_size/2])

colorbar

xlabel('X/m')
ylabel('Y/m')
