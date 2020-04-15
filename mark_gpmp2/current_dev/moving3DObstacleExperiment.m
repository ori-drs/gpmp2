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

% v_or_t_end_value = [0,0, 0];
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
[X, Y, Z] = getEnvironmentMesh(dataset);

% plot problem setting
figure(1); hold on;
title('3D Environment')
grid on, view(3)
gpmp2.set3DPlotRange(dataset);
xlabel('x'); ylabel('y'); zlabel('z');
plot3DEnvironment(dataset, X, Y, Z)
plotArm(arm.fk_model(), start_conf, 'b', 2)
plotArm(arm.fk_model(), end_conf, 'r', 2)

% figure(2);
% gpmp2.set3DPlotRange(dataset);
% grid on, view(3)
% for t = 0:0.5:5
%     env.updateMap(t);
%     dataset = env.getDataset();
%     cla;
%     plot3DEnvironment(dataset, X, Y, Z)
%     plotArm(arm.fk_model(), start_conf, 'b', 2)
%     plotArm(arm.fk_model(), end_conf, 'r', 2)    
%     pause(0.05);
% end



%% Planner settings
total_time_sec = 5.0;
delta_t = 0.1;
total_time_step = round(total_time_sec/delta_t);
total_check_step = 10*total_time_step;
check_inter = total_check_step / total_time_step - 1;
pause_time = delta_t;

% use GP interpolation
use_GP_inter = true;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.02;
epsilon_dist = 0.1;

% noise model
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

%% isam

% settings
opt_setting = TrajOptimizerSetting(7);
opt_setting.set_total_step(total_time_step);
opt_setting.set_total_time(total_time_sec);
opt_setting.set_epsilon(epsilon_dist);
opt_setting.set_cost_sigma(cost_sigma);
opt_setting.set_obs_check_inter(check_inter);
opt_setting.set_conf_prior_model(pose_fix_sigma);
opt_setting.set_vel_prior_model(vel_fix_sigma);
opt_setting.set_Qc_model(Qc);

% initial values by batch
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
ori_traj_values = BatchTrajOptimize3DArm(arm, dataset.sdf, start_conf, start_vel, end_conf, ...
    end_vel, init_values, opt_setting);

% isam init
isam_planner = ISAM2TrajOptimizer3DArm(arm, dataset.sdf, opt_setting);
% initial graph
isam_planner.initFactorGraph(start_conf, start_vel, end_conf, end_vel);
% insert original traj values
isam_planner.initValues(ori_traj_values);
% one update let isam accept original values
isam_planner.update();

figure(4)
title('Result Values')
grid on, view(3) %view(-83, 86)
hold on

% plot world
% plotMap3D(dataset.corner_idx, origin, dataset.cell_size);

% first plann results
for i=0:total_time_step
    % plot arm
    conf = ori_traj_values.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 1);
    % plot config
    set3DPlotRange(dataset);
    pause(pause_time)
end
