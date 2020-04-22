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
t_start_moving = 0;
v_or_t_end = true;
use_all_straight_initialisations = false;

% v_or_t_end_value = [0,0, 0];
v_or_t_end_value = [0,-0.08, 0];
starting_pos = [0.40, 0.2, 0.4];
obs_size = [0.1, 0.1, 0.1];

use_trustregion_opt = false;


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

if plot_figs
    % plot problem setting
    figure(1); hold on;
    title('3D Environment')
    grid on, view(3)
    gpmp2.set3DPlotRange(dataset);
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3DEnvironment(dataset, X, Y, Z)
    plotArm(arm.fk_model(), start_conf, 'b', 2)
    plotArm(arm.fk_model(), end_conf, 'r', 2)

    figure(2);
    gpmp2.set3DPlotRange(dataset);
    grid on, view(3)
    for t = 0:0.5:5
        env.updateMap(t);
        dataset = env.getDataset();
        cla;
        plot3DEnvironment(dataset, X, Y, Z)
        plotArm(arm.fk_model(), start_conf, 'b', 2)
        plotArm(arm.fk_model(), end_conf, 'r', 2)    
        pause(0.05);
    end

end

%% Planner settings
total_time_sec = 5.0;
delta_t = 0.25;
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

pose_fix_model = noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, vel_fix_sigma);

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
tic
start_sdf = env.queryEnv(0).sdf;
toc
%% build graph

disp('Case1: Static sdf');

static_graph = NonlinearFactorGraph;

for i = 0:total_time_step
    pose_key = gtsam.symbol('x', i);
    vel_key = gtsam.symbol('v', i);

    % start and end
    if i == 0
      static_graph.add(PriorFactorVector(pose_key, start_conf, pose_fix_model));
      static_graph.add(PriorFactorVector(vel_key, start_vel, vel_fix_model));

    elseif i == total_time_step
      static_graph.add(PriorFactorVector(pose_key, end_conf, pose_fix_model));
      static_graph.add(PriorFactorVector(vel_key, end_vel, vel_fix_model));
    end

    % non-interpolated cost factor
    static_graph.add(ObstacleSDFFactorArm(pose_key, ...
                                        arm, ...
                                        start_sdf, ...
                                        cost_sigma, ...
                                        epsilon_dist));

    if i > 0
        last_pose_key = gtsam.symbol('x', i-1);
        last_vel_key = gtsam.symbol('v', i-1);

      % interpolated cost factor
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter 
                tau = j * (total_time_sec / total_check_step);
                static_graph.add(ObstacleSDFFactorGPArm( ...
                    last_pose_key, last_vel_key, pose_key, vel_key, ...
                    arm, start_sdf, ...
                    cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end

      % GP factor
        static_graph.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                last_vel_key, ...
                                                pose_key, ...
                                                vel_key, ...
                                                delta_t, ...
                                                Qc_model));
        
    end
end

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    static_optimizer = DoglegOptimizer(static_graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    static_optimizer = GaussNewtonOptimizer(static_graph, init_values, parameters);
end


static_optimizer.optimize();
static_result = static_optimizer.values();

%%

full_knowledge_graph = NonlinearFactorGraph;

obs_fact_indices = zeros(1, total_time_step+1);

factor_ind_counter = 0;
for i = 0:total_time_step
    pose_key = gtsam.symbol('x', i);
    vel_key = gtsam.symbol('v', i);

    t = i *  delta_t;
    dataset = env.queryEnv(t);
    
    % start and end
    if i == 0
      full_knowledge_graph.add(PriorFactorVector(pose_key, start_conf, pose_fix_model));
      full_knowledge_graph.add(PriorFactorVector(vel_key, start_vel, vel_fix_model));
      factor_ind_counter = factor_ind_counter + 2;
    elseif i == total_time_step
      full_knowledge_graph.add(PriorFactorVector(pose_key, end_conf, pose_fix_model));
      full_knowledge_graph.add(PriorFactorVector(vel_key, end_vel, vel_fix_model));
      factor_ind_counter = factor_ind_counter + 2;

    end

    % non-interpolated cost factor
    obs_fact_indices(i+1) = factor_ind_counter;
    full_knowledge_graph.add(ObstacleSDFFactorArm(pose_key, ...
                                        arm, ...
                                        dataset.sdf, ...
                                        cost_sigma, ...
                                        epsilon_dist));
    factor_ind_counter = factor_ind_counter + 1;

    if i > 0
        last_pose_key = gtsam.symbol('x', i-1);
        last_vel_key = gtsam.symbol('v', i-1);

      % interpolated cost factor
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter 
                tau = j * (total_time_sec / total_check_step);
                full_knowledge_graph.add(ObstacleSDFFactorGPArm( ...
                    last_pose_key, last_vel_key, pose_key, vel_key, ...
                    arm, dataset.sdf, ...
                    cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                factor_ind_counter = factor_ind_counter + 1;

            end
        end

      % GP factor
        full_knowledge_graph.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                last_vel_key, ...
                                                pose_key, ...
                                                vel_key, ...
                                                delta_t, ...
                                                Qc_model));
        factor_ind_counter = factor_ind_counter + 1;

    end
end


if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    full_optimizer = DoglegOptimizer(full_knowledge_graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    full_optimizer = GaussNewtonOptimizer(full_knowledge_graph, init_values, parameters);
end


full_optimizer.optimize();
full_result = full_optimizer.values();


%% Plot case 1
datasets = [];
for i = 0:total_time_step
    t = i *  delta_t;
    env.updateMap(t);
    dataset = env.getDataset();
    datasets = [datasets, dataset];
end    
figure(3);
hold on;
% gpmp2.set3DPlotRange(dataset);
axis([0 1 -0.5 0.5 0 1]);
grid on, view(3)
for i = 0:total_time_step
%     t = i *  delta_t;
%     env.updateMap(t);
%     dataset = env.getDataset();
    static_conf = static_result.atVector(symbol('x', i));
    full_conf = full_result.atVector(symbol('x', i));
%     cla;
    plot3DEnvironment(datasets(i+1), X, Y, Z)
    plotArm(arm.fk_model(), static_conf, 'b', 2)
    plotArm(arm.fk_model(), full_conf, 'r', 2)
%     plotRobotModel(arm, conf)
    pause(0.05);
end

% 
% figure(4);
% hold on;
% axis([0 1 -0.5 0.5 0 1]);
% grid on, view(3)
% conf = result.atVector(gtsam.symbol('x', 10));
% plot3DEnvironment(datasets(11), X, Y, Z)
% gpmp2.plotArm(arm.fk_model(), conf, 'b', 2)
% 
% mid_factor = full_knowledge_graph.at(102);
% 
% for i = 1:numel(obs_fact_indices)
%     ind = obs_fact_indices(i);
%     sum(full_knowledge_graph.at(ind).evaluateError(conf))
% end

% For each time step
for i = 0:total_time_step
    %for each obstacle factor
    for j = 1:total_time_step+1
        ind = obs_fact_indices(j);
        fact = full_knowledge_graph.at(ind);
        fact.error(full_result)
%         fact.error(static_result)

    end
end

    
