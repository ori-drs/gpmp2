% 
% @author Mark Finean 
% @date April 13, 2020

close all;
clear all;
clc;
% profile on

import gtsam.*
import gpmp2.*

plot_figs = false;

%% Setup

env_name = 'MovingReplanner';
env = loadPredefinedMovingEnvironment('MovingReplanner');
dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);
use_trustregion_opt = false;

%% Problem setup

replanner_start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
up_conf = [0,0,0,0,0,0,0]';
replanner_end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';

start_conf = replanner_start_conf;
end_conf = replanner_end_conf;
start_vel = zeros(7,1);
end_vel = zeros(7,1);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

%% Planner settings
total_time_sec = 1.0;
delta_t = 0.1;
total_time_step = round(total_time_sec/delta_t);
interp_multiplier = 20;
total_check_step = interp_multiplier*total_time_step;
check_inter = total_check_step / total_time_step - 1;
pause_time = delta_t;

% use GP interpolation
use_GP_inter = true;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.1;
epsilon_dist = 0.1;

% noise model
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

pose_fix_model = noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, vel_fix_sigma);

problem_setup1.start_conf = start_conf;
problem_setup1.end_conf = up_conf;
problem_setup1.start_vel = start_vel;
problem_setup1.end_vel = end_vel;
problem_setup1.total_time_step = total_time_step;
problem_setup1.total_time_sec = total_time_sec;
problem_setup1.total_check_step = total_check_step;
problem_setup1.delta_t = delta_t;
problem_setup1.check_inter = check_inter;
problem_setup1.pose_fix_sigma = pose_fix_sigma;
problem_setup1.pose_fix_model = pose_fix_model;
problem_setup1.vel_fix_sigma = vel_fix_sigma;
problem_setup1.vel_fix_model = vel_fix_model;
problem_setup1.cost_sigma = cost_sigma;
problem_setup1.epsilon_dist = epsilon_dist;
problem_setup1.arm = arm;
problem_setup1.Qc_model = Qc_model;
problem_setup1.use_trustregion_opt = use_trustregion_opt;

problem_setup2.start_conf = up_conf;
problem_setup2.end_conf = end_conf;
problem_setup2.start_vel = start_vel;
problem_setup2.end_vel = end_vel;
problem_setup2.total_time_step = total_time_step;
problem_setup2.total_time_sec = total_time_sec;
problem_setup2.total_check_step = total_check_step;
problem_setup2.delta_t = delta_t;
problem_setup2.check_inter = check_inter;
problem_setup2.pose_fix_sigma = pose_fix_sigma;
problem_setup2.pose_fix_model = pose_fix_model;
problem_setup2.vel_fix_sigma = vel_fix_sigma;
problem_setup2.vel_fix_model = vel_fix_model;
problem_setup2.cost_sigma = cost_sigma;
problem_setup2.epsilon_dist = epsilon_dist;
problem_setup2.arm = arm;
problem_setup2.Qc_model = Qc_model;
problem_setup2.use_trustregion_opt = use_trustregion_opt;


% initial values by batch
init_values1 = initArmTrajStraightLine(start_conf, up_conf, total_time_step);
init_values2 = initArmTrajStraightLine(up_conf, end_conf, total_time_step);

if strcmp(env_name, 'MovingReplanner')
    axis_lims = [-1 1.5 -1.2 1.5 -1 2];
else
    axis_lims = [-0.5 1 -0.5 0.5 -0.2 1];
end


%% get datasets

disp('Getting all the datasets');

datasets = [];

for i = 0:total_time_step*2
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

%% build graphs

full_knowledge_case1 = case2(datasets(1:total_time_step+1), init_values1, problem_setup1);
static_case1 = case1(datasets(1).sdf, init_values1, problem_setup1);
full_knowledge_case2 = case2(datasets(total_time_step+1:end), init_values2, problem_setup2);
static_case2 = case1(datasets(1).sdf, init_values2, problem_setup2);


%% Plot 
import gtsam.*
import gpmp2.*

figure(4);
hold on;
axis(axis_lims);
grid on; view(3);
title("Full knowledge");
xlabel('x'); ylabel('y'); zlabel('z');


obs_error = 0;
     
for i = 0:total_time_step
    ind = static_case1.obs_fact_indices(i+1);
    static_conf = static_case1.result.atVector(gtsam.symbol('x', i));
    fact = static_case1.graph.at(ind);
%     cla;
    if i > 0
        delete(h1);
    end
    h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);

    static_conf = static_case1.result.atVector(symbol('x', i));
    static_handle = plotArm(arm.fk_model(), static_conf, 'g', 2);
%         plotRobotModel(arm, full_knowledge_conf);

    disp("Step: " + num2str(i));
    disp(fact.evaluateError(static_conf));
    
    pause(0.2);

end

for i = 0:total_time_step
    ind = static_case2.obs_fact_indices(i+1);
    static_conf = static_case2.result.atVector(gtsam.symbol('x', i));
    fact = static_case2.graph.at(ind);
%     cla;
    if i > 0
        delete(h1);
    end
    h1 = plot3DEnvironment(datasets(i+1+total_time_step), X, Y, Z);

    static_conf = static_case2.result.atVector(symbol('x', i));
    static_handle = plotArm(arm.fk_model(), static_conf, 'b', 2);
%         plotRobotModel(arm, full_knowledge_conf);

    disp("Step: " + num2str(i));
    disp(fact.evaluateError(static_conf));
    
    pause(0.2);

end

%% Plot 
import gtsam.*
import gpmp2.*

figure(5);
hold on;
axis(axis_lims);
grid on; view(3);
title("Full knowledge");
xlabel('x'); ylabel('y'); zlabel('z');


obs_error = 0;
     
for i = 0:total_time_step
    ind = full_knowledge_case1.obs_fact_indices(i+1);
    conf = full_knowledge_case1.result.atVector(gtsam.symbol('x', i));
    fact = full_knowledge_case1.graph.at(ind);
%     cla;
    if i > 0
        delete(h1);
    end
    h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);

    full_knowledge_conf = full_knowledge_case1.result.atVector(symbol('x', i));
    full_knowledge_handle = plotArm(arm.fk_model(), full_knowledge_conf, 'g', 2);
%         plotRobotModel(arm, full_knowledge_conf);

    disp("Step: " + num2str(i));
    disp(fact.evaluateError(conf));
    
    pause(0.2);

end

for i = 0:total_time_step
    ind = full_knowledge_case2.obs_fact_indices(i+1);
    conf = full_knowledge_case2.result.atVector(gtsam.symbol('x', i));
    fact = full_knowledge_case2.graph.at(ind);
%     cla;
    if i > 0
        delete(h1);
    end
    h1 = plot3DEnvironment(datasets(i+1+total_time_step), X, Y, Z);

    full_knowledge_conf = full_knowledge_case2.result.atVector(symbol('x', i));
    full_knowledge_handle = plotArm(arm.fk_model(), full_knowledge_conf, 'g', 2);
%         plotRobotModel(arm, full_knowledge_conf);

    disp("Step: " + num2str(i));
    disp(fact.evaluateError(full_knowledge_conf));
    
    pause(0.2);

end
