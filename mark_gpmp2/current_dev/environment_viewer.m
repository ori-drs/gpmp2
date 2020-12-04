% 
% @author Mark Finean 
% @date Oct 20, 2020

close all;
clear all;
clc;

import gtsam.*
import gpmp2.*

node = ros.Node('/env_viewer_node');

%% Setup
% Setup ROS interactions
cell_size = 0.04;
env_size = 64;
origin = [-1,-1,-1];
obstacle = "ar_box";
scene = "big_room"; 

env = realEnvironment(node, env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();
env.updateMap() % Load the static map into map

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

%% arm model
arm = gpmp2.generateArm('Panda');
arm_model = arm.fk_model();

% Setup problem
start_conf = setPandaConf('right_ready');
end_conf = setPandaConf('table_forward');

start_vel = zeros(7,1);
end_vel = zeros(7,1);
hold on;
% gpmp2.plotArm(arm.fk_model(), start_conf, 'b', 2)
% gpmp2.plotArm(arm.fk_model(), end_conf, 'r', 2)
gpmp2.plotRobotModel(arm, start_conf)
% gpmp2.plotRobotModel(arm, end_conf)
view(3);


