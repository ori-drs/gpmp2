% A test script for verifying obstacle costs are as expoected
% @author Mark Finean 
% @date April 15, 2020

close all;
clear;
clc;

import gtsam.*
import gpmp2.*

%% Define the main arm positions
arm = generateArm('WAMArm');
arm_model = arm.fk_model();


% Problem setup
up_conf = [0,0,0,0,0,0,0]';
down_conf = [0,3.14,0,0,0,0,0]';
left_conf = [-1.57,1.57,0,0,0,0,0]';
right_conf = [1.57,1.57,0,0,0,0,0]';
forward_conf = [0,1.57,0,0,0,0,0]';
backward_conf = [0,-1.57,0,0,0,0,0]';

num_steps = 9;
conf = up_conf + i * (forward_conf - up_conf) / num_steps;
arm_confs = [];
figure(1);
for i = 0:num_steps-1
    conf = up_conf + i * (forward_conf - up_conf) / num_steps;
    subplot(3,3,i+1)
    hold on; grid on; view(3);
    title("Step: " + num2str(i));
    plotArm(arm.fk_model(), conf, 'r', 2);
    xlim([-0.2,1]); ylim([-0.5,0.5]); zlim([-0.1,1]);
    arm_confs = [arm_confs, conf];
    pause(0.05);
end

%% Defined all Environments
obs_size = [0.2, 0.2, 0.2];

starting_pos = [0.3,-0.5,0.3];
block_v = [0,1/num_steps,0];
            

env = movingEnvironment3D(300,0.01);
env.add_object(0,...
                0, ...
                block_v, ...
                starting_pos, ...
                obs_size);
dataset = env.queryEnv(0);


% Get a mesh from one of the datasets        
[X, Y, Z] = getEnvironmentMesh(dataset);

datasets = [];
for i = 0:num_steps-1
    dataset = env.queryEnv(i);
    datasets = [datasets, dataset];
end
            
%% plot the environments
figure(2)
sgtitle("Environments")

for i = 0:num_steps-1
    conf = up_conf + i * (forward_conf - up_conf) / num_steps;
    subplot(3,3,i+1); hold on; grid on; view(3);
    plot3DEnvironment(datasets(i+1), X, Y, Z);
    title("Step: " + num2str(i));
    xlim([-0.2,1]); ylim([-0.5,0.5]); zlim([-0.1,1]);
    xlabel('x'); ylabel('y'); zlabel('z');
    pause(0.05);
end

%% Calculating the costs

cost_sigma = 0.9;
epsilon_dist = 0.01;

key_pos = gtsam.symbol('x', 0);
key_vel = gtsam.symbol('v', 0);

arm_values = [];
for i = 1:size(arm_confs,2)
    init_values = gtsam.Values;
    init_values.insert(key_pos, arm_confs(:,i));
    init_values.insert(key_vel, [0, 0, 0, 0, 0, 0, 0]');
    arm_values = [arm_values, init_values];
end


% For each environment
for j = 0:num_steps-1
    figure(j+1); hold on;
    sgtitle("Environment: " + num2str(j));
    
    % Create obstacle factor for the environment
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(j+1).sdf, cost_sigma, epsilon_dist);
        
    for i = 1:size(arm_confs,2)
       
        % Calculate the error
        error = obs_factor.error(arm_values(i));
        
        subplot(3,3,i); hold on; grid on, view(3);
        plot3DEnvironment(datasets(j+1), X, Y, Z);
        plotArm(arm.fk_model(), arm_confs(:,i), 'b', 2);
        title("Arm step " + num2str(i-1) + " - Error: " + num2str(error));
        xlim([-0.2,1]); ylim([-0.5,0.5]); zlim([-0.1,1]);
        xlabel('x'); ylabel('y'); zlabel('z');   
    end
    pause(1);
end
