% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all
clear

import gtsam.*
import gpmp2.*

%%
t_start_moving = 0;
v_or_t_end = true;
v_or_t_end_value = [0,0];
% v_or_t_end_value = [1,-1];

%% settings 
total_time_sec = 4.0;
% delta_t = 0.05;
delta_t = 0.1;
total_time_step = round(total_time_sec/delta_t);
total_check_step = 10*total_time_step;
check_inter = total_check_step / total_time_step - 1;
pause_time = delta_t;

% total_time_sec = 10.0;
% total_time_step = 10;
% total_check_step = 100;
% delta_t = total_time_sec / total_time_step;
% check_inter = total_check_step / total_time_step - 1;
% pause_time = total_time_sec / total_time_step;

% use GP interpolation
use_GP_inter = true;

% arm model
arm = generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();

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

% start and end conf
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

%% Plot the start of the problem
dataset = generateMovingScene(t_start_moving, v_or_t_end, v_or_t_end_value, 0);

rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

figure(1), hold on
customPlotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title('Layout')
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
hold off

% for i=0:total_time_step
%     query_t = i *delta_t;
%     figure(1)
%     dataset = generateMovingScene(t_start_moving, v_or_t_end, v_or_t_end_value, i);
%     customPlotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     pause(0.1)
% end



%% init optimization

graph = NonlinearFactorGraph;
obs_graph = NonlinearFactorGraph;

init_values = Values;

datasets = cell(total_time_step,1);

for i = 0 : total_time_step
    query_t = i * delta_t;
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    datasets{i+1} = generateMovingScene(t_start_moving, v_or_t_end, v_or_t_end_value, query_t);
    % initialize as straight line in conf space
    pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
    vel = avg_vel;
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
    
    % start/end priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));

        % cost factor
        graph.add(ObstaclePlanarSDFFactorArm(...
            key_pos, arm, datasets{i+1}.sdf, cost_sigma, epsilon_dist));
        obs_graph.add(ObstaclePlanarSDFFactorArm(...
            key_pos, arm, datasets{i+1}.sdf, cost_sigma, epsilon_dist));
                
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, datasets{i+1}.sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                obs_graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, datasets{i+1}.sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end



%% optimize!
use_trustregion_opt = false;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

optimizer.optimize();
result = optimizer.values();

conf_list = [];
positions = [];
for i=0:total_time_step
    conf = result.atVector(symbol('x', i));
    positions = [positions, arm_model.forwardKinematicsPosition(conf)];
    conf_list = [conf_list,conf];
end

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', obs_graph.error(init_values))

optimizer.optimize();

result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', obs_graph.error(result))



%% plot final values
import gtsam.*
import gpmp2.*

conf_list = [];
positions = [];
for i=0:total_time_step
    conf = result.atVector(symbol('x', i));
    positions = [positions, arm_model.forwardKinematicsPosition(conf)];
    conf_list = [conf_list,conf];
end
conf_list = wrapToPi(conf_list);

% positions = confToPositions();
% 
for i=0:total_time_step
    figure(4), hold on
    title('Optimized Values')
    customPlotEvidenceMap2D(datasets{i+1}.map, datasets{i+1}.origin_x, datasets{i+1}.origin_y, cell_size);
    customPlotPlanarArm(arm.fk_model(), conf_list(:,i+1), 'b', 2);
    pause(pause_time*2), hold off
end

% figure(6), hold on
% visualiseTrajectory(arm.fk_model(), conf_list)
% xlim([dataset.origin_x, dataset.origin_x + dataset.cols*dataset.cell_size]);
% ylim([dataset.origin_y, dataset.origin_y + dataset.rows*dataset.cell_size]);
% 


