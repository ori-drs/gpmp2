% planar arm obstacle avoidance example with moving obstacle, build factor graph in matlab
% @author Mark Finean (adapted from Jing Dong)
% @date Mar 18, 2020

close all
clear

import gtsam.*
import gpmp2.*

%% settings
total_time_sec = 10.0;
total_time_step = 50;
total_check_step = 100;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = true;
%% small dataset
% Want to start at 190 and move across to 
dataset = generate2Dmovingdataset('OneObstacleDataset', total_time_step); % y, x position
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

% plot sdf
% figure(1)
% plotSignedDistanceField2D(dataset.fields{1}, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')

% arm model
arm = generateArm('SimpleTwoLinksArm');

% GP
Qc = eye(2);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% Obstacle avoid settings
cost_sigma = 0.05;
epsilon_dist = 0.01;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

% start and end conf
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% plot start / end configuration
% figure(2), hold on
% plotEvidenceMap2D(dataset.maps{1}, dataset.origin_x, dataset.origin_y, cell_size);
% title('Layout')
% plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
% % plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
% hold off
% 
% figure(3), hold on
% plotEvidenceMap2D(dataset.maps{11}, dataset.origin_x, dataset.origin_y, cell_size);
% title('Layout')
% % plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
% plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
% hold off


%% init optimization
graph = NonlinearFactorGraph;
graph_obs = NonlinearFactorGraph;

init_values = Values;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
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
            key_pos, arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist));
        graph_obs.add(ObstaclePlanarSDFFactorArm(...
            key_pos, arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist));
                
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                graph_obs.add(ObstaclePlanarSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

% %% plot initial values
% for i=0:total_time_step
%     figure(3), hold on
%     title('Initial Values')
%     % plot world
%     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     conf = init_values.atVector(symbol('x', i));
%     plotPlanarArm(arm, conf, 'b', 2);
%     pause(pause_time), hold off
% end


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

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', graph_obs.error(init_values))

optimizer.optimize();

result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', graph_obs.error(result))




conf_x_series = [];
conf_v_series = [];

% plot final values
% figure(4)
% for i=0:total_time_step
%     
%     time_steps = 0:delta_t:i*delta_t;
% 
%     x_conf = result.atVector(symbol('x', i));
%     conf_x_series = horzcat(conf_x_series, x_conf);
%     v_conf = result.atVector(symbol('v', i));
%     conf_v_series = horzcat(conf_v_series, v_conf);
%     
%     figure(4), hold on
%     title('Optimized Values')
%     % plot world
%     plotEvidenceMap2D(dataset.maps{i+1}, dataset.origin_x, dataset.origin_y, cell_size);
%     % plot arm
%     conf = result.atVector(symbol('x', i));
%     plotPlanarArm(arm.fk_model(), conf, 'b', 2);
% %     hold off
%     
%     figure(5);
%     hold on;
%     plot(time_steps, conf_x_series(1,:), 'r')
%     plot(time_steps, conf_x_series(2, :), 'b')
% 
%     plot(time_steps, conf_v_series(1, :), 'r--')
%     plot(time_steps, conf_v_series(2, :), 'b--')
% 
%     legend('x1','x2','v1','v2')
%     xlabel('Time (s)');
%     ylabel('State value');
%     xlim([0,total_time_step*delta_t]);
%     ylim([-6,+6]);
% 
%     pause(pause_time)
% end


%%

obs_poses = dataset.obs_poses;
obs_sizes = dataset.obs_sizes;
origin_x = dataset.origin_x;
origin_y = dataset.origin_y;

% For each obstacles
dataset.origin_x = -1;
    dataset.origin_y = -1;

f4 = figure(4);
ax = gca; % current axes
for i=0:total_time_step
    block_pos_x = [obs_poses{i+1}(1); obs_poses{i+1}(1)+obs_sizes{i+1}(1); ...
    obs_poses{i+1}(1)+obs_sizes{i+1}(1); obs_poses{i+1}(1)];

    block_pos_y = [obs_poses{i+1}(2); obs_poses{i+1}(2); ...
        obs_poses{i+1}(2)+obs_sizes{i+1}(2); obs_poses{i+1}(2)+obs_sizes{i+1}(2)];

    % Add Origin
    block_pos_x = block_pos_x + origin_x/cell_size;
    block_pos_y = block_pos_y + origin_y/cell_size;

    
    c = [1];

%     clf(4)
    patch(block_pos_x * cell_size, ...
    block_pos_y * cell_size, ...
    c,...
    'FaceAlpha', i*0.9/size(obs_poses,2));
    colormap(gray(100));
    hold on
    conf = result.atVector(symbol('x', i));
    customPlotPlanarArm(arm.fk_model(), wrapToPi(conf), 'b', 2); 
    xlabel('X/m');
    ylabel('Y/m');
    axis equal
    axis([origin_x, cols*cell_size, ...
    origin_y, rows*cell_size])
    pause(0.5);
end


time_steps = 0:delta_t:total_time_sec;
conf_x_series = [];
conf_v_series = [];

for i=0:total_time_step
    x_conf = result.atVector(symbol('x', i));
    conf_x_series = horzcat(conf_x_series, x_conf);
    v_conf = result.atVector(symbol('v', i));
    conf_v_series = horzcat(conf_v_series, v_conf);
end
figure(6);
hold on;
plot(time_steps, conf_x_series)
plot(time_steps, conf_v_series)
legend('x1','x2','v1','v2')
xlabel('Time (s)');
ylabel('State value');