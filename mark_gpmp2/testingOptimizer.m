% planar arm obstacle avoidance example, build factor graph in matlab
% @author Jing Dong
% @date Nov 16, 2015

close all
clear

import gtsam.*
import gpmp2.*


%% small dataset
% dataset = generate2DdatasetMark('OneObstacleDataset');
dataset = generateMovingScene('Static');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
% field = signedDistanceField2D(dataset.map, cell_size);
% sdf = PlanarSDF(origin_point2, cell_size, field);

% plot sdf
figure(1)
plotSignedDistanceField2D(flip(dataset.field), dataset.origin_x, dataset.origin_y, dataset.cell_size);
title('Signed Distance Field');
hold off;
pause(0.1);

%% settings
total_time_sec = 5.0;
total_time_step = 25;
total_check_step = 125;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = true;

% arm model
arm = generateArm('SimpleTwoLinksArm');
arm_model = arm.fk_model();

% GP
Qc = eye(2);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% Obstacle avoid settings
cost_sigma = 0.01;
epsilon_dist = 0.1;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

% joint velocity limit param
flag_joint_vel_limit = false;
joint_vel_limit_vec = [1, 1]';
joint_vel_limit_thresh = 0.01 * ones(2,1);
joint_vel_limit_model = noiseModel.Isotropic.Sigma(2, 0.1);

% start and end conf
start_conf = [0, 0]';
start_vel = [0, 0]';
% end_conf = [-3*pi/2, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% plot start / end configuration
figure(2), hold on
plotEvidenceMap2D(flip(dataset.map), dataset.origin_x, dataset.origin_y, cell_size);
title('Layout')
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
hold off;
pause(0.1);


%% init optimization
graph = NonlinearFactorGraph;
obs_graph = NonlinearFactorGraph;

init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initialize as straight line in conf space
    pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
    vel = avg_vel;
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
    
    % joint velocity limit factor on every velocity
    if flag_joint_vel_limit
        graph.add(VelocityLimitFactorVector(key_vel, joint_vel_limit_model, ...
            joint_vel_limit_vec, joint_vel_limit_thresh));
    end    
    
    % start/end priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
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
            key_pos, arm, dataset.sdf, cost_sigma, epsilon_dist));
        obs_graph.add(ObstaclePlanarSDFFactorArm(...
            key_pos, arm, dataset.sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                obs_graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
    
    if i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
end

%% optimize!
import gtsam.*
import gpmp2.*

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

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', obs_graph.error(init_values))

optimizer.optimize();

result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', obs_graph.error(result))


%% plot final values
% for i=0:total_time_step
%     figure(4), hold on
%     title('Optimized Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     conf = result.atVector(symbol('x', i));
%     plotPlanarArm(arm.fk_model(), conf, 'b', 2);
%     pause(pause_time), hold off
% end

%% optimize!
import gtsam.*
import gpmp2.*

parameters = GaussNewtonParams;
parameters.setVerbosity('ERROR');
optimizer = GaussNewtonOptimizer(graph, init_values, parameters);


%% Iterative approach
import gtsam.*
import gpmp2.* 

plot_bool = false;
iters = 50;
num_steps = double(size(result)/2-1);
% num_plot_cols = double(ceil(double(num_steps)/double(num_plot_rows)));
plot_step_size = 5;
plots = (iters/plot_step_size);
num_plot_cols = 5;
num_plot_rows = ceil(double(plots)/num_plot_cols);

format bank;
plot_counter = 0;
results = {};
for k=0:iters-1
    result = optimizer.values();
    results{k+1} = result;
    error = graph.error(result);
    obs_error = obs_graph.error(result);

    if mod(k, plot_step_size) == 0 & plot_bool
        plot_counter = plot_counter +1;
        % Plotting a trajectory  
        figure(3), hold on;
        cmap = colormap(parula(total_time_step+1));
        h1 = subplot(num_plot_rows, num_plot_cols, plot_counter);
%         ax1 = gca; % current axes

        title('Iteration:' + string(k) + ...
            " Error:" + num2str(error,'% 10.2f'))

        for i=0:total_time_step

            conf = result.atVector(symbol('x', i));
            position = arm_model.forwardKinematicsPosition(wrapToPi(conf));

            block_pos_x = [dataset.obs_pose(1)-dataset.obs_size(1)/2; ...
                            dataset.obs_pose(1)+dataset.obs_size(1)/2; ...
                            dataset.obs_pose(1)+dataset.obs_size(1)/2; ...
                            dataset.obs_pose(1)-dataset.obs_size(1)/2];

            block_pos_y = [dataset.obs_pose(2)-dataset.obs_size(2)/2; ...
                            dataset.obs_pose(2)-dataset.obs_size(2)/2; ...
                            dataset.obs_pose(2)+dataset.obs_size(2)/2; ...
                            dataset.obs_pose(2)+dataset.obs_size(2)/2];

            % Add Origin
%             block_pos_x = block_pos_x + dataset.origin_x/dataset.cell_size;
%             block_pos_y = block_pos_y + dataset.origin_y/dataset.cell_size;

%             c = [1];
            c = [0.25, 0.25, 0.25]; % grey

        %     clf(4)
            patch(h1, block_pos_x , block_pos_y, ...
            c,...
            'FaceAlpha', 1);
%             colormap(gray(100));
            hold on;
            scatterPlotPlanarArm(position, cmap(i+1,:), 0, (i+1)*0.9/total_time_step, h1); 
    %         customPlotPlanarArm(arm.fk_model(), wrapToPi(conf), 'b', 2); 
            xlabel('X/m');
            ylabel('Y/m');
            axis equal
            axis([dataset.origin_x, dataset.cols*dataset.cell_size, ...
            dataset.origin_y, dataset.rows*dataset.cell_size])
            pause(0.05);
        end
        hold off

        pause(0.5)
       
        figure(4);
        hold on;
        h2 = subplot(num_plot_rows, num_plot_cols, plot_counter);
%         ax2 = gca; % current axes
        plotStateEvolution(result, delta_t, total_time_sec, total_time_step, h2)
        title('Iteration:' + string(k+1))
        hold off

        pause(0.5);
        
        
    end
    optimizer.iterate();    

end