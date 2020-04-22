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
t_start_moving = 0;
v_or_t_end = true;
use_all_straight_initialisations = false;

v_or_t_end_value = [0,-0.08, 0];
starting_pos = [0.40, 0.2, 0.4];
obs_size = [0.1, 0.1, 0.1];

%% Problem setup
up_conf = [0,0,0,0,0,0,0]';
forward_conf = [0,1.57,0,0,0,0,0]';

start_conf = up_conf;
end_conf = forward_conf;
start_vel = zeros(7,1);
end_vel = zeros(7,1);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

%% Create the environment
env = movingEnvironment3D();
env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value, ...
                starting_pos, ...
                obs_size);
   
dataset = env.queryEnv(0);

% sdf1 = dataset.sdf;
% sdf1.initFieldData(0, dataset.field(:,:,1)');
% sdf1.initFieldData(0, env.queryEnv(1).field(:,:,1)');

[X, Y, Z] = getEnvironmentMesh(dataset);

%% plot problem setting
% figure(1); hold on;
% sgtitle('3D Environment')
% for i=0:5
%     dataset = env.queryEnv(i);
%     subplot(2,3,i+1);
%     hold on;
%     title("Time " + num2str(i));
%     grid on, view(0, 90)
%     gpmp2.set3DPlotRange(dataset);
%     xlabel('x'); ylabel('y'); zlabel('z');
%     plot3DEnvironment(dataset, X, Y, Z);
%     % plotArm(arm.fk_model(), start_conf, 'b', 2)
%     % plotArm(arm.fk_model(), end_conf, 'r', 2)
%     pause(1);
% end



key_pos1 = gtsam.symbol('x', 0);
cost_sigma = 0.1;
epsilon_dist = 0.1;
fact = gpmp2.DynamicObstacleSDFFactorArm(key_pos1, ...
                                    arm, ...
                                    env.queryEnv(0).sdf, ...
                                    cost_sigma, ...
                                    epsilon_dist);

graph = gtsam.NonlinearFactorGraph;
init_values = gtsam.Values;
key_pos = gtsam.symbol('x', 0);
graph.add(fact);

new_sdf = env.queryEnv(1).sdf;

% graph.at(0).print('');
tic
new_fact = gpmp2.DynamicObstacleSDFFactorArm(key_pos1, ...
                                    arm, ...
                                    new_sdf, ...
                                    cost_sigma, ...
                                    epsilon_dist);
                                
graph.replace(0, new_fact);
toc

% % 
disp("it worked");
