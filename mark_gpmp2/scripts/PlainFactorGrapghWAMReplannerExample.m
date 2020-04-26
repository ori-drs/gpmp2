% 7DOF WAM arm replanning example
% @author Jing Dong


close all
clear

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% arm: WAM arm
arm = generateArm('WAMArm');

% plan conf
start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% replan conf
replan_pose_idx = 5;
replan_end_conf = [-0.6,0.94,0,1.6,0,-0.919,1.55]';

% plot problem setting
figure(1), hold on
title('Problem Settings')
plotMap3D(dataset.corner_idx, origin, cell_size);
plotArm(arm.fk_model(), start_conf, 'r', 2)
plotArm(arm.fk_model(), end_conf, 'b', 2)
plotArm(arm.fk_model(), replan_end_conf, 'g', 2)
% plot config
set3DPlotRange(dataset)
grid on, view(3)
hold off


%% settings
total_time_sec = 5;
total_time_step = 10;
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
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

pose_fix_model = noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, vel_fix_sigma);


% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% plot settings
plot_inter_traj = true;
plot_inter = 5;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
    plot_inter = 0;
end
pause_time = total_time_sec / total_plot_step;


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


%% Graph

inc_graph_ = NonlinearFactorGraph;


% build graph
for i = 0:total_time_step
    pose_key = gtsam.symbol('x', i);
    vel_key = gtsam.symbol('v', i);

    % start and end
    if i == 0
      inc_graph_.add(PriorFactorVector(pose_key, start_conf, pose_fix_model));
      inc_graph_.add(PriorFactorVector(vel_key, start_vel, vel_fix_model));

    elseif i == total_time_step
      inc_graph_.add(PriorFactorVector(pose_key, end_conf, pose_fix_model));
      end_conf_factor_idx_ = inc_graph_.size() - 1;        % cache goal factor index here
      inc_graph_.add(PriorFactorVector(vel_key, end_vel, vel_fix_model));
      end_vel_factor_idx_ = inc_graph_.size() - 1;         % cache goal factor index here
      goal_removed_ = false;
    end

    % non-interpolated cost factor
    inc_graph_.add(ObstacleSDFFactorArm(pose_key, ...
                                        arm, ...
                                        sdf, ...
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
                
                inc_graph_.add(ObstacleSDFFactorGPArm( ...
                    last_pose_key, last_vel_key, pose_key, vel_key, ...
                    arm, sdf, ...
                    cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end

      % GP factor
        inc_graph_.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                last_vel_key, ...
                                                pose_key, ...
                                                vel_key, ...
                                                delta_t, ...
                                                Qc_model));
        
    end
end

% ori_traj_values = BatchTrajOptimize3DArm(arm, sdf, start_conf, start_vel, end_conf, ...
%     end_vel, init_values, opt_setting);
use_trustregion_opt = false;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(inc_graph_, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(inc_graph_, init_values, parameters);
end
tic
optimizer.optimize();
result = optimizer.values();
toc

figure(4)
hold on;
title('Result Values')
grid on, view(3) %view(-83, 86)
% plot world
plotMap3D(dataset.corner_idx, origin, dataset.cell_size);

% first plann results
for i=0:total_time_step
    % plot arm
    conf = result.atVector(symbol('x', i));
    plotArm(arm.fk_model(), conf, 'b', 1);
    % plot config
    set3DPlotRange(dataset);
    pause(0.5)
end


%% replanning
% 
% % fix currernt conf and vel, update replanned end conf and vel
% curr_conf = ori_traj_values.atVector(symbol('x', replan_pose_idx));
% curr_vel = ori_traj_values.atVector(symbol('v', replan_pose_idx));
% isam_planner.fixConfigAndVel(replan_pose_idx, curr_conf, curr_vel);
% % update replanned end conf and vel
% isam_planner.changeGoalConfigAndVel(replan_end_conf, end_vel);
% % optimize! 
% isam_planner.update();
% % get results
% replan_result = isam_planner.values();
