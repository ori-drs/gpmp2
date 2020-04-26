% 
% @author Mark Finean 
% @date April 13, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*
%% Setup
env = loadPredefinedMovingEnvironment('MovingReplanner');

dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);

% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
axis([-1 1.5 -1.2 1.5 -1 2]);
xlabel('x');
ylabel('y');
zlabel('z');
plot3DEnvironment(dataset, X, Y, Z, [0.5 0.5 0.5])

%% arm model
arm = generateArm('WAMArm');
arm_model = arm.fk_model();

% Problem setup
start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);
hold on;
plotArm(arm.fk_model(), start_conf, 'b', 2)
plotArm(arm.fk_model(), end_conf, 'r', 2)
