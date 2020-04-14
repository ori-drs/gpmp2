% 7DOF WAM arm example, build factor graph in matlab, with moving
% obstacles
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 19, 2020

close all
clear
clc

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Dmovingdataset('MovingBox', [0,-4,0], 18); %shift = [y,x,z]. 
% Hand far right = [0,38,0]
% Hand far left = [0,-18,0]

cell_size = dataset.cell_size;

% sdf
% disp('calculating signed distance field ...');
% field = signedDistanceField3D(dataset.map, dataset.cell_size);
% disp('calculating signed distance field done');

% arm: WAM arm
arm = generateArm('WAMArm');

% start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
start_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% plot problem setting
figure(1), hold on
title('Problem Settings')
grid on, view(3)
set3DPlotRange(dataset)
plotMap3DMoving(dataset.corner_idxs{1}, dataset.origin, cell_size);
plotRobotModel(arm, start_conf)
plotRobotModel(arm, end_conf)
% plot config

hold off


%% settings
total_time_sec = 2;
total_time_step = 18;
total_check_step = 100;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.02;
epsilon_dist = 0.2;

% noise model
fix_sigma = 0.0001;
pose_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);

% plot settings
plot_inter_traj = false;
plot_inter = 4;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;


%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

% % plot initial traj
% if plot_inter_traj
%     plot_values = interpolateArmTraj(init_values, Qc_model, delta_t, plot_inter);
% else
%     plot_values = init_values;
% end
% 
% % plot init values
% figure(3),
% hold on
% title('Initial Values')
% % plot world
% plotMap3D(dataset.corner_idx, dataset.origin, cell_size);
% for i=0:total_plot_step
%     % plot arm
%     conf = plot_values.atVector(symbol('x', i));
%     plotPhysicalArm(arm, conf)
%     % plot config
%     set3DPlotRange(dataset)
%     grid on, view(2)
%     pause(pause_time)
% end
% hold off


%% init optimization
graph = NonlinearFactorGraph;
graph_obs = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
   
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix_model));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix_model));
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
        graph.add(ObstacleSDFFactorArm(...
            key_pos, arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist));
        graph_obs.add(ObstacleSDFFactorArm(...
            key_pos, arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                graph_obs.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

%% optimize!
use_LM = true;
use_trustregion_opt = false;

if use_LM
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
%     parameters.setVerbosityLM('LAMBDA');
    parameters.setlambdaInitial(1000.0);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
elseif use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', graph_obs.error(init_values))

optimizer.optimize();

result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', graph_obs.error(result))


%% plot results
import gtsam.*
import gpmp2.*
if plot_inter_traj
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end

pause(pause_time)
% plot final values
f1 = figure(4),
title('Result Values')
clf, hold on
% plot world
% plotMap3D(dataset.corner_idxs{i+1}, dataset.origin, cell_size);
plotMap3DMoving(dataset.corner_idxs{1}, dataset.origin, dataset.cell_size,1);
plotMap3DMoving(dataset.corner_idxs{total_plot_step+1}, dataset.origin, dataset.cell_size, total_plot_step+1);
for i=0:total_plot_step
    % plot arm
    
%     plotMap3D(dataset.corner_idxs{i+1}, dataset.origin, dataset.cell_size);
    conf = plot_values.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 2)
    % plot config
    set3DPlotRange(dataset)
    grid on, view(-80, 12) %view(-5, 12)
    pause(pause_time)
end
hold off
pause(pause_time)

%%
% plot final values
import gtsam.*
import gpmp2.*
f = figure(5);
for i=0:total_plot_step
    clf
    hold on, view(-5, 12)
    title('Maintain Pose with Moving Box')
    % plot world
    plotMap3D(dataset.corner_idxs{i+1}, dataset.origin, dataset.cell_size);
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    plotRobotModel(arm, conf)
    % plot config
    set3DPlotRange(dataset)
    grid on, view(-80, 12)
    pause(pause_time)
end
hold off


%%
import gtsam.*
import gpmp2.*

figure(7),
clf, hold on
title('Result Values')
for i=0:total_plot_step
    plotMap3D(dataset.corner_idxs{i+1}, dataset.origin, dataset.cell_size);
%     conf = plot_values.atVector(symbol('x', i));
%     plotArm(arm.fk_model(), conf, 'b', 2)
    % plot config
    set3DPlotRange(dataset)
    grid on, view(0, 90) %view(-5, 12)
    pause(pause_time)
end
hold off
pause(pause_time)

