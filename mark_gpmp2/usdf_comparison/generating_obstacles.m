clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Parameters
cell_size = 0.04;
env_size = 64;
origin = [-1.28,-1.28,-1.28];

%% Environment

% Obstacle def
num_obstacles = 1000;

obstacle_set = rand(num_obstacles,6);
obstacle_set(:, 1:3) = (obstacle_set(:, 1:3) .* [2, 2, 1]) - [1,1,0]; % Random numbers between -1 and 1 for x,y and 0-1 for z

e = 1;

% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object(obstacle_set(e, 1:3), obstacle_set(e, 4:6));
env.updateMap();


%% Task

% Setup problem
conf = setPandaConf('right_ready');


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

h1 = plot3DEnvironment(dataset, X, Y, Z);

rot = eye(3);
base_pos = [0,0,0];
arm = generateArm('Panda', Pose3(Rot3(rot), Point3(base_pos')));

static_handle = gpmp2.plotRobotModel(arm, conf);


