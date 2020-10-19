% 
% @author Mark Finean 
% @date Oct 12, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*

%% arm model
arm = gpmp2.generateArm('Kinova');
arm_model = arm.fk_model();

% Problem setup

start_conf = [0.5,0.5,0,0,0,0]';
% start_conf = [4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233]';
start_vel = zeros(6,1);
end_vel = zeros(6,1);
hold on;
gpmp2.plotRobotModel(arm, start_conf)
view(3);
axis 'equal'
% plotArm(arm.fk_model(), end_conf, 'r', 2)
