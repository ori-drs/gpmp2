% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all;
clear;
clc;

import gtsam.*
import gpmp2.*

%% Set up the environment
t_start_moving = 0;
v_or_t_end = true;
v_or_t_end_value = [0,0];

env = movingEnvironment(0,true,[0,0]);
dataset = env.queryEnv(0);

%% settings 
total_time_sec = 4.0;
delta_t = 0.1;
total_time_step = round(total_time_sec/delta_t);
total_check_step = 10*total_time_step;
check_inter = total_check_step / total_time_step - 1;
pause_time = delta_t;

% use GP interpolation
use_GP_inter = true;

%% Set up the robot task
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

% arm model
arm = generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();

% joint velocity limit param
flag_joint_vel_limit = false;
joint_vel_limit_vec = [1, 1]';
joint_vel_limit_thresh = 0.01 * ones(2,1);
joint_vel_limit_model = noiseModel.Isotropic.Sigma(2, 0.1);
%% GP and task setup
% GP
Qc = eye(2);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% Obstacle avoid settings
cost_sigma = 0.05;
epsilon_dist = 0.2;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

%% Plot the start of the problem

% Set up the factor graph
[graph,obs_graph, init_values] = createFactorGraph(dataset, start_conf,end_conf,start_vel, end_vel, avg_vel,...
                                    total_time_sec, total_check_step, total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix);

[graph_2, obs_graph_2, init_values] = createFactorGraph(dataset, start_conf,end_conf,start_vel, end_vel, avg_vel,...
                                    total_time_sec, total_check_step, total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix);
                                
[graph_3,obs_graph_3, init_values] = createFactorGraph(dataset, start_conf,end_conf,start_vel, end_vel, avg_vel,...
                                    total_time_sec, total_check_step, total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix);
                                
[graph_4,obs_graph_4, init_values] = createFactorGraph(dataset, start_conf,end_conf,start_vel, end_vel, avg_vel,...
                                    total_time_sec, total_check_step, total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix);
                                                                
%% Initialise the optimizer

parameters = GaussNewtonParams;
parameters.setVerbosity('ERROR');

init_optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
case_2_optimizer = GaussNewtonOptimizer(graph_2, init_values, parameters);
case_3_optimizer = GaussNewtonOptimizer(graph_3, init_values, parameters);
case_4_optimizer = GaussNewtonOptimizer(graph_4, init_values, parameters);

%%

% Loop through timesteps
query_t = 1;
dataset = env.queryEnv(query_t);


    % Update optimisation
    
    
    
%% Iterative approach
% import gtsam.*
% import gpmp2.* 

case_2_results = {};

% Need three cases: 
%     1) no further updates, 
%     2) update all future ones to the currently observed sdf
%     3) update future sdfs based on prediction
%     4) update future sdfs based on prediction ONLY if relevant

% Case 1 - this does not need to be in the loop as it does not account for
% changes
init_optimizer.optimize();
init_result = init_optimizer.values();

% Cases 2,3,4
for i = 0 : total_time_step
    query_t = i * delta_t;
    
    % Get dataset for current timestep (making and observation) 
    dataset_t = env.queryEnv(query_t);
    
    % Case 2 - we change all future sdfs to the newly observed one SDF prediction
    
    
    % Case 3 -update future sdfs based on their predictions
    if i>1
        % Get the change in SDF from last
        field_change = dataset_t.field - last_field; 
        % Note this is how we access the sdf in latlab. 
        % We then need to change class for GPMP2
        
        predicted_field  = gpmp2.signedDistanceField2D(obj.dataset.map, obj.dataset.cell_size);
        dataset_t.sdf = gpmp2.PlanarSDF(dataset_t.origin_point2, dataset_t.cell_size, dataset_t.field);
        
        inds_ignore = sdf_change>0.99*min(min(sdf_change));
        
%         Calculate the time for coillision for each voxel
        time_to_collision = -dataset_t.sdf./sdf_change;
        time_to_collision(time_to_collision<0)=10;
        
        full_time_to_collision = time_to_collision;
        time_to_collision(inds_ignore)=10;

    end    
    
    % Case 4 -update future sdfs based on their predictions ONLY if
    % relevant

    
    
    % Updating factors
    
    
    % Store run optimisation and get values
    case_2_optimizer.optimize();
    case_2_result = case_2_optimizer.values();

    case_3_optimizer.optimize();
    case_3_result = case_2_optimizer.values();
    
    case_4_optimizer.optimize();
    case_4_result = case_2_optimizer.values();
    
    % Store all the resulting trajectories
    case_1_results{i+1} = init_result;
    case_2_results{i+1} = case_2_result;
    case_3_results{i+1} = case_3_result;
    case_4_results{i+1} = case_4_result;

    % store last sdf
    last_field = dataset_t.field;
end