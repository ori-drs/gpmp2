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

figure(1);
subplot(3,2,1)
hold on;
title('Up');
plotArm(arm.fk_model(), up_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

subplot(3,2,2)
hold on;
title('Down')
plotArm(arm.fk_model(), down_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

subplot(3,2,3)
hold on;
title('Left')
plotArm(arm.fk_model(), left_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

subplot(3,2,4)
hold on;
title('Right')
plotArm(arm.fk_model(), right_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

subplot(3,2,5)
hold on;
title('Forward')
plotArm(arm.fk_model(), forward_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

subplot(3,2,6)
hold on;
title('Backward')
plotArm(arm.fk_model(), backward_conf, 'r', 2);
grid on, view(3)
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

%% Defined all Environments
obs_size = [0.2, 0.2, 0.2];

starting_pos = [0,0,0];
            

up_env = movingEnvironment3D();
up_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0,0,0.9], ...
                obs_size);
up_dataset = up_env.queryEnv(0);

down_env = movingEnvironment3D();
down_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0,0,-0.9], ...
                obs_size);
down_dataset = down_env.queryEnv(0);

left_env = movingEnvironment3D();
left_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0,-0.9,0], ...
                obs_size);
left_dataset = left_env.queryEnv(0);

right_env = movingEnvironment3D();
right_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0,0.9,0], ...
                obs_size);
right_dataset = right_env.queryEnv(0);

origin_env = movingEnvironment3D();
origin_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0,0,0], ...
                obs_size);
origin_dataset = origin_env.queryEnv(0);

forward_env = movingEnvironment3D();
forward_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [0.9,0,0], ...
                obs_size);
forward_dataset = forward_env.queryEnv(0);

backward_env = movingEnvironment3D();
backward_env.add_object(0,...
                true, ...
                [0,0,0], ...
                [-0.9,0,0], ...
                obs_size);hold on;

backward_dataset = backward_env.queryEnv(0);



% Get a mesh from one of the datasets        
[X, Y, Z] = getEnvironmentMesh(backward_dataset);


arm_confs = [up_conf, down_conf, left_conf, ...
             right_conf, forward_conf, backward_conf];
         
datasets = [up_dataset, down_dataset, left_dataset, ...
            right_dataset, forward_dataset, backward_dataset];
        
title_order = ["up", "down", "left", ...
                "right", "forward", "backward"];
            
            
%% plot the environments
figure(2)
sgtitle("Environments")

subplot(3,2,1); hold on;
plot3DEnvironment(up_dataset, X, Y, Z)
title('Up');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(90,0);

subplot(3,2,2); hold on;
plot3DEnvironment(down_dataset, X, Y, Z)
title('Down');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(90,0);

subplot(3,2,3); hold on;
plot3DEnvironment(left_dataset, X, Y, Z)
title('Left');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(90,0);

subplot(3,2,4); hold on;
plot3DEnvironment(right_dataset, X, Y, Z)
title('Right');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(90,0);

subplot(3,2,5); hold on;
plot3DEnvironment(forward_dataset, X, Y, Z)
title('Forward');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(0,0);

subplot(3,2,6); hold on;
plot3DEnvironment(backward_dataset, X, Y, Z)
title('Backward');
xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
xlabel('x'); ylabel('y'); zlabel('z');
view(0,0);

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
view_azimuthal_order = [90, 90, 90, 90 ,0, 0];
for j = 1:6   
    figure(j+2);
    hold on;
    sgtitle('Environment ' + title_order(j));
    
    % Create obstacle factor for the environment
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(j).sdf, cost_sigma, epsilon_dist);
        
    for i = 1:size(arm_confs,2)
       
        % Calculate the error
        error = obs_factor.error(arm_values(i));
        
        subplot(3,2,i); hold on;
        plot3DEnvironment(datasets(j), X, Y, Z);
        plotArm(arm.fk_model(), arm_confs(:,i), 'b', 2);
        title("Arm " + title_order(i) + "-Error: " + num2str(error));
        xlim([-1,1]); ylim([-1,1]); zlim([-1,1]);
        xlabel('x'); ylabel('y'); zlabel('z');
%         view(view_azimuthal_order(i),0);
        grid on, view(3);
    end
    pause(1);
end
