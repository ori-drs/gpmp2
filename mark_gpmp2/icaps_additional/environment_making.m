clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

%% Environment
cell_size = 0.02;
env_size = 128;
origin = [-1.28,-1.28,-1.28];

% Obstacle def
base_pos = [0, 0, 0];    
rot = eye(3);
arm = generateArm('Panda', Pose3(Rot3(rot), Point3(base_pos')));


% Create the environment
env = loadComparisonEnvironment(env_size, cell_size, origin);
env.add_object([1.0, -0.35, 0], [0.9, 0.3, 0.8]);
env.add_object([0.4, -0.35, -0.1], [0.4, 0.3, 0.6]);
env.add_object([0.0, -0.35, 1.0], [3.0, 0.3, 0.6]);

env.add_object([0, -0.35, -0.15], [0.4, 0.3, 0.54]);

env.add_object([0.0, -0.35, 0.6], [0.6, 0.3, 0.2]);

% env.add_object([1.0, -0.35, 0.35], [1.0, 0.3, 1]);


env.add_object([-1.0, -0.35, 0.35], [1.4, 0.3, 1]);

% env.add_object(starting_pos1, obs_size);
env.updateMap();

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

h1 = plot3DEnvironment(dataset.map, X, Y, Z);

%%
% conf =  [0, 1.2, 0.1, -0.2, 1.57, 3, 1.14]';
% conf =  [1.3, 0.44, 0.11, -1.52, 0.1, 1.36, 0.6]';
conf =  [-1.8, 1.5, 0.5, -0.2, 1.57, 2, 1.0]';

static_handle = gpmp2.plotRobotModel(arm, conf);
