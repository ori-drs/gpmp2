clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

scene = "big_room"; % "bookshelf" 
obstacle = "Tag0";

cell_size = 0.04;
env_size = 64;
origin = [-1,-1,-1];

% Setup ROS interactions
node = ros.Node('/matlab_node');

env = realEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

start_sdf = env.getSDF();

env.updateMap();

%%
% env.updateMap();

dataset = env.getDataset();
[X, Y, Z] = getEnvironmentMesh(dataset);

% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
axis([-1 1.5 -1.2 1.5 -1 2]);
xlabel('x');
ylabel('y');
zlabel('z');
plot3DEnvironment(dataset, X, Y, Z, 0.5)

arm = gpmp2.generateArm('Panda');
arm_model = arm.fk_model();

% Setup problem
start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');

start_vel = zeros(7,1);
end_vel = zeros(7,1);
hold on;

gpmp2.plotRobotModel(arm, start_conf)
gpmp2.plotRobotModel(arm, end_conf)
view(3);
