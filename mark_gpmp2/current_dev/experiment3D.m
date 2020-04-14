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

v_or_t_end_value = [0.2,-0.2, 0];
starting_pos = [-0.40, 1.90, 1.5];
obs_size = [0.60, 0.80, 0.60];

%% Create the environment
env = movingEnvironment3D(0,...
                            v_or_t_end, ...
                            v_or_t_end_value, ...
                            starting_pos, ...
                            obs_size);
           
dataset = env.queryEnv(0);

[X, Y, Z] = getEnvironmentMesh(dataset);

% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
gpmp2.set3DPlotRange(dataset);
xlabel('x');
ylabel('y');
zlabel('z');
for t = 0:0.5:5
    env.updateMap(t);
    dataset = env.getDataset();
    cla;
    plot3DEnvironment(dataset, X, Y, Z)
    
    pause(0.05);
end

% %% Planner settings
% total_time_sec = 5.0;
% delta_t = 0.1;
% total_time_step = round(total_time_sec/delta_t);
% total_check_step = 10*total_time_step;
% check_inter = total_check_step / total_time_step - 1;
% pause_time = delta_t;
% 
% % use GP interpolation
% use_GP_inter = true;
% 
% %% Problem setup
% start_conf = [0, 0]';
% start_vel = [0, 0]';
% end_conf = [pi/2, 0]';
% end_vel = [0, 0]';
% 
% % arm model
% arm = generateArm('SimpleTwoLinksArm');
% arm_model = arm.fk_model();

%% Initial plots
% % % plot sdf
% figure(1)
% plotSignedDistanceField2D(dataset.field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field');
% hold off;
% pause(0.1);
% % 
% % % plot start / end configuration
% figure(2), hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Layout')
% plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
% plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
% hold off;
% pause(0.1);

