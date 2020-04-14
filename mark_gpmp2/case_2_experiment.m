% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all;
clear;
clc;

import gtsam.*
import gpmp2.*

t_start_moving = 0;
v_or_t_end = true;
use_all_straight_initialisations = false;

% v_or_t_end_value = [0.2,-0.2];
% starting_pos = [-0.50, 1.90];
v_or_t_end_value = [0,-0.2];
starting_pos = [0.40, 0.6];
obs_size = [0.60, 0.80];

env = movingEnvironment(0,v_or_t_end,v_or_t_end_value, starting_pos, obs_size);
dataset = env.queryEnv(0);

total_time_sec = 6.0;
delta_t = 0.1;
total_time_step = round(total_time_sec/delta_t);
total_check_step = 10*total_time_step;
check_inter = total_check_step / total_time_step - 1;
pause_time = delta_t;

% use GP interpolation
use_GP_inter = true;

start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

% arm model
arm = generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();


% % plot sdf
figure(1)
plotSignedDistanceField2D(dataset.field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
title('Signed Distance Field');
hold off;
pause(0.1);
% 
% % plot start / end configuration
figure(2), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, dataset.cell_size);
title('Layout')
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
hold off;
pause(0.1);

% joint velocity limit param
flag_joint_vel_limit = false;
joint_vel_limit_vec = [1, 1]';
joint_vel_limit_thresh = 0.01 * ones(2,1);
joint_vel_limit_model = noiseModel.Isotropic.Sigma(2, 0.1);
% GP
Qc = eye(2);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% Obstacle avoid settings
cost_sigma = 0.9;
epsilon_dist = 0.0001;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

% Set up the factor graph
[graph, obs_graph , obs_factor_inds] = createFactorGraph(dataset, start_conf,end_conf,start_vel, end_vel,...
                                    total_time_sec, total_check_step, total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix);

                                
if use_all_straight_initialisations                                
    init_values = getAllStraightLineInitialisations(start_conf, end_conf, total_time_step, delta_t);
else
    init_values = getStraightLineInitialisation(start_conf, end_conf, total_time_step, delta_t);
end

% Initialise the optimizer

parameters = GaussNewtonParams;
parameters.setVerbosity('ERROR');
    
results = cell(1,total_time_step + 1);
fields = cell(1,total_time_step + 1);
obs_poses = cell(1,total_time_step + 1);
collision_costs = zeros(total_time_step+1,total_time_step + 1);

for i = 0 : total_time_step

    query_t = i * delta_t;
    dataset_t = env.queryEnv(query_t);
    obs_poses{i+1} = dataset_t.obs_pose;
    field = dataset_t.field;
    
    % Update all future factors
    for k = 1:size(obs_factor_inds,2)
        time = obs_factor_inds(2,k);
        if time>i
            fact_ind = obs_factor_inds(1,k);
            graph.at(fact_ind).changeSDFData(field);
        end
    end
    
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);

    % Store run optimisation and get values
    optimizer.optimize();
    result = optimizer.values();
    
    collision_cost_vector = getCollisionErrorVector(graph, obs_factor_inds, result);
    
    init_values = result;
    % Store all the resulting trajectories
    results{i+1} = result;
    fields{i+1} = field;
    collision_costs(i+1,:) = collision_cost_vector;
end


% Animation
iters = total_time_sec/delta_t + 1;
f = figure(3);
obs_size = dataset_t.obs_size;
pause_time = 0.0;

% trajectory_and_sdf_slider(results, delta_t, total_time_step, ...
%                                     iters, fields, dataset_t.origin_x, ...
%                                     dataset_t.origin_y, dataset_t.cell_size,...
%                                     obs_poses, obs_size, arm, f)

trajectory_and_block_slider(results, delta_t, ...
                        iters, dataset_t.cell_size,...
                        [dataset_t.origin_x,dataset_t.origin_y],...
                        obs_poses, obs_size, arm, pause_time, f)



for i = obs_factor_inds(1,:)
    graph.at(i)
end

obs_errors = [];
for i = 1:size(results, 2)
    err = obs_graph.error(results{i});
    obs_errors = [obs_errors, err];
end
        