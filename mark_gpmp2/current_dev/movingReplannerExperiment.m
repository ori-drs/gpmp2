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
env = loadPredefinedMovingEnvironment(env_name);
dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);
use_trustregion_opt = false;

%% Problem setup

replanner_start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
replanner_end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';

start_conf = replanner_start_conf;
end_conf = replanner_end_conf;
start_vel = zeros(7,1);
end_vel = zeros(7,1);

arm = generateArm('WAMArm');
arm_model = arm.fk_model();

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
total_time_sec = 3.0;
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
cost_sigma = 0.05;
epsilon_dist = 0.2;

% noise model
pose_fix_sigma = 0.0001;
vel_fix_sigma = 0.0001;

pose_fix_model = noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, vel_fix_sigma);

problem_setup.start_conf = start_conf;
problem_setup.end_conf = end_conf;
problem_setup.start_vel = start_vel;
problem_setup.end_vel = end_vel;
problem_setup.total_time_step = total_time_step;
problem_setup.total_time_sec = total_time_sec;
problem_setup.total_check_step = total_check_step;
problem_setup.delta_t = delta_t;
problem_setup.check_inter = check_inter;
problem_setup.pose_fix_sigma = pose_fix_sigma;
problem_setup.pose_fix_model = pose_fix_model;
problem_setup.vel_fix_sigma = vel_fix_sigma;
problem_setup.vel_fix_model = vel_fix_model;
problem_setup.cost_sigma = cost_sigma;
problem_setup.epsilon_dist = epsilon_dist;
problem_setup.arm = arm;
problem_setup.Qc_model = Qc_model;
problem_setup.use_trustregion_opt = use_trustregion_opt;

% initial values by batch
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

start_sdf = env.queryEnv(0).sdf;

if strcmp(env_name, 'MovingReplanner')
    axis_lims = [-1 1.5 -1.2 1.5 -1 2];
else
    axis_lims = [-0.5 1 -0.5 0.5 -0.2 1];
end


%% get datasets

disp('Getting all the datasets');

datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

%% build graphs
import gtsam.*
import gpmp2.*

disp('Case1: Static sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
static_case = case1(start_sdf, init_values, problem_setup);

disp('Case2: Full knowledge sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
full_knowledge_case = case2(datasets, init_values, problem_setup);

disp('Case3: Execute and update sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
execute_update_case = case3(datasets, init_values, problem_setup);

% disp('Case4: Execute and selectively predict sdf');
% init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
% selective_prediction_case = case4(datasets, init_values, problem_setup, true);

% disp('Case5: Execute and selectively update sdf');
% init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
% collision_t_update_case = case5(datasets, init_values, problem_setup);

% This should be the same result as full knowledge but MUCH slower
% disp('Case6: Execute and predict sdf');
% init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
% prediction_case = case6(datasets, init_values, problem_setup, true);
disp('Case7: Execute and update sdf using reinitialisation');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
execute_update_case_reinit = case7(datasets, init_values, problem_setup);

disp('Case8: Execute and update sdf using pruning');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
pruning_case = case8(datasets, init_values, problem_setup);

%% Plot the comparison animation
import gtsam.*
import gpmp2.*

figure(3);
hold on;
% gpmp2.set3DPlotRange(dataset);
axis(axis_lims);
% grid on; view(45,45);
grid on; view(3);
for i = 0:total_time_step
    static_conf = static_case.result.atVector(symbol('x', i));
    full_conf = full_knowledge_case.result.atVector(symbol('x', i));
    execute_update_conf = execute_update_case.final_result.atVector(symbol('x', i));
    selective_prediction_conf =selective_prediction_case.final_result.atVector(symbol('x', i));
%     collision_t_update_conf =collision_t_update_case.final_result.atVector(symbol('x', i));
%     cla;
    if i > 0
        delete(h1);
    end
    h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
    static_handle = plotArm(arm.fk_model(), static_conf, 'b', 2);
    full_handle = plotArm(arm.fk_model(), full_conf, 'r', 2);
    execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'g', 2);
    selective_prediction_handle = plotArm(arm.fk_model(), selective_prediction_conf, 'm', 2);
%     collision_t_update_handle = plotArm(arm.fk_model(), collision_t_update_conf, 'c', 2);
    
%     plotRobotModel(arm, conf)
    legend([static_handle(1), full_handle(1), ... 
        execute_update_handle(1), selective_prediction_handle(1)], ...
        {"No SDF update", "Full knowledge", ...
        "Update each step", "Selective prediction"},...
        'Location','southoutside',...
        'NumColumns', 3, ...
        'FontSize', 16);
    pause(0.2);
end


%% Plot the static graph
import gtsam.*
import gpmp2.*

figure(4);
hold on;
axis(axis_lims);
grid on; 
% view(3);
view(-1.007130428616419e+02, 50.314206527540222);
campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
title("Static Graph");
xlabel('x'); ylabel('y'); zlabel('z');


% figure(5);
% hold on;
% axis(axis_lims);
% grid on; 
% % view(3);
% view(-1.007130428616419e+02, 50.314206527540222);
% % campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
% title("Static Graph");
% xlabel('x'); ylabel('y'); zlabel('z');

for i = 0:total_time_step
    key_pos = symbol('x', i);
    static_conf = static_case.result.atVector(key_pos);
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(i+1).sdf, cost_sigma, epsilon_dist);
    
    
    figure(4);
%     cla;
%     if i > 0
%         delete(h1);
%     end
%     h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);

    if any(obs_factor.spheresInCollision(static_conf))
        static_handle = plotArm(arm.fk_model(), static_conf, 'r', 2);
    else
        static_handle = plotArm(arm.fk_model(), static_conf, 'b', 2);
    end
    
    
%     figure(5);
%     cla;
%     h2 = plot3DEnvironment(datasets(i+1), X, Y, Z);
%     plotRobotModel(arm, static_conf);

    
%     ind = full_knowledge_case.obs_fact_indices(j);
%     fact = full_knowledge_case.graph.at(ind);
        
%         disp(fact.evaluateError(conf));
%         obs_error = obs_error + sum(fact.evaluateError(conf));


%     static_handle = plotArm(arm.fk_model(), static_conf, 'b', 2);

    pause(0.01);

end

% 
% function collision_bool = collisionCheck(dataset, conf)
%     joint_positions = arm_model.forwardKinematicsPosition(conf);
%     joint_positions(:, [1 2]) = joint_positions(:, [2 1]);
%     joint_coords = positionToCoord(joint_positions, ...
%                                 [dataset.origin_x, dataset.origin_y, dataset.origin_z], ...
%                                 dataset.rows, dataset.cell_size);
%     workspace_size = [dataset.rows, dataset.cols, dataset.z];
%     
%     query_inds = sub2ind(workspace_size,...
%                         joint_coords(:,1)', ...
%                         joint_coords(:,2)', ...
%                         joint_coords(:,3)');
% 
%     for i = 1:size(joint_positions,2)
%         arm.nr_body_spheres
%     end                                
% end
% 
% 
% function swapped_coord = positionToCoord(positions, origin, rows, cell_size)
%     positions(:,2) = -positions(:,2);
%     origin(:,2) = - origin(:,2);
% 
%     coord = round((positions - origin)/cell_size) + [1, rows ,1];
%     swapped_coord = zeros(size(coord));
%     swapped_coord(:,1) = coord(:,2);
%     swapped_coord(:,2) = coord(:,1);
%     swapped_coord(:,3) = coord(:,3);
% end
%% Plot the full knowledge
import gtsam.*
import gpmp2.*

figure(5);
hold on;
axis(axis_lims);
grid on; 
% view(3);
view(-1.007130428616419e+02, 50.314206527540222);
campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
title("Full knowledge");
xlabel('x'); ylabel('y'); zlabel('z');

figure(6);
hold on;
axis(axis_lims);
grid on; view(3);
title("Full knowledge");
xlabel('x'); ylabel('y'); zlabel('z');


obs_error = 0;
figure(5);
 
for i = 0:total_time_step
    
    key_pos = symbol('x', i);
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(i+1).sdf, cost_sigma, epsilon_dist);

    ind = full_knowledge_case.obs_fact_indices(i+1);
    conf = full_knowledge_case.result.atVector(key_pos);
    fact = full_knowledge_case.graph.at(ind);
    
    cla;
%     if i > 0
%         delete(h1);
%     end
%     figure(5);
    h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);

    plotRobotModel(arm, conf);

%     disp("Step: " + num2str(i));
%     disp(fact.evaluateError(conf));
%     
%     if any(obs_factor.spheresInCollision(conf))
%         full_knowledge_handle = plotArm(arm.fk_model(), conf, 'r', 2);
%     else
%         full_knowledge_handle = plotArm(arm.fk_model(), conf, 'b', 2);
%     end
    
%     figure(6);
%     cla;
%     h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
%     plotRobotModel(arm, conf);

    
    pause(0.2);

end

% 
% figure(10);
% ax = gca;
% plotStateEvolution(full_knowledge_case.result, delta_t, total_time_sec, total_time_step, ax, 'x')
% figure(11);
% ax = gca;
% plotStateEvolution(full_knowledge_case.result, delta_t, total_time_sec, total_time_step, ax, 'v')

%% Plot the execute update evolution
import gtsam.*
import gpmp2.*

figure(6);
subplot(2,1,1);
hold on;
axis(axis_lims);
grid on; view(3);
title("Execute and Update");
xlabel('x'); ylabel('y'); zlabel('z');

subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

obs_factor_steps = 0:total_time_step;
obs_interp_factor_steps = setdiff(0:1/interp_multiplier:total_time_step, obs_factor_steps);

for j = 0:total_time_step

    subplot(2,1,1);
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        key_pos = symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                key_pos, arm, datasets(i+1).sdf, cost_sigma, epsilon_dist);

    
        execute_update_conf = execute_update_case.results(j+1).atVector(key_pos);
        in_collision_bool = any(obs_factor.spheresInCollision(execute_update_conf));
        if i>j
            execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'b', 2);
        elseif in_collision_bool &&  i<=j % executed and collides
            execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'r', 2);
        else  % executed 
            execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'g', 2);
        end
    end
    
    subplot(2,1,2); 
    factor_times_updated = execute_update_case.update_timings.factors_steps_updated{j+1};
    
    if ~isempty(factor_times_updated)
        all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                        + [0:interp_multiplier-1] ,1, [])/interp_multiplier;
                                    
        updated_interps = setdiff(all_updated_factors, obs_factor_steps);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);

    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_steps, ones(size(obs_interp_factor_steps)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_steps, ones(size(obs_factor_steps)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.2);
end

%% Plot the pruning and reinit evolution
import gtsam.*
import gpmp2.*

figure(6);
subplot(2,1,1);
hold on;
axis(axis_lims);
grid on; view(3);
title("Prune and Reinit in Danger");
xlabel('x'); ylabel('y'); zlabel('z');

subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

obs_factor_steps = 0:total_time_step;
obs_interp_factor_steps = setdiff(0:1/interp_multiplier:total_time_step, obs_factor_steps);

for j = 0:total_time_step

    subplot(2,1,1);
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        key_pos = symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                key_pos, arm, datasets(i+1).sdf, cost_sigma, epsilon_dist);

    
        prune_reinit_conf = pruning_case.results(j+1).atVector(key_pos);
        in_collision_bool = any(obs_factor.spheresInCollision(prune_reinit_conf));
        if i>j
            prune_reinit_handle = plotArm(arm.fk_model(), prune_reinit_conf, 'b', 2);
        elseif in_collision_bool &&  i<=j % executed and collides
            prune_reinit_handle = plotArm(arm.fk_model(), prune_reinit_conf, 'r', 2);
        else  % executed 
            prune_reinit_handle = plotArm(arm.fk_model(), prune_reinit_conf, 'g', 2);
        end
    end
    
    subplot(2,1,2); 
    factor_times_updated = pruning_case.update_timings.factors_steps_updated{j+1};
    
    if ~isempty(factor_times_updated)
        all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                        + [0:interp_multiplier-1] ,1, [])/interp_multiplier;
                                    
        updated_interps = setdiff(all_updated_factors, obs_factor_steps);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);

    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_steps, ones(size(obs_interp_factor_steps)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_steps, ones(size(obs_factor_steps)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.2);
end

%% Plot the pruning robot
import gtsam.*
import gpmp2.*

figure(7);
hold on;
axis(axis_lims);
grid on; 
% view(3);
view(-1.007130428616419e+02, 50.314206527540222);
campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
title("Prune and Reinit");
xlabel('x'); ylabel('y'); zlabel('z');

for j = 0:total_time_step
    key_pos = symbol('x', j);
    
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(j+1).sdf, cost_sigma, epsilon_dist);

    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    prune_reinit_conf = pruning_case.results(end).atVector(key_pos);
    
    plotRobotModel(arm, prune_reinit_conf);
 
    pause(0.2);
end

%% Plot the execute update evolution
import gtsam.*
import gpmp2.*

figure(7);
hold on;
axis(axis_lims);
grid on; 
% view(3);
view(-1.007130428616419e+02, 50.314206527540222);
campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
title("Execute and Update");
xlabel('x'); ylabel('y'); zlabel('z');

for j = 0:total_time_step
    key_pos = symbol('x', j);
    
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, arm, datasets(j+1).sdf, cost_sigma, epsilon_dist);

    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    execute_update_conf = execute_update_case.results(end).atVector(key_pos);
%     
%     if any(obs_factor.spheresInCollision(execute_update_conf))
%         execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'r', 2);
%     else
%         execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'b', 2);
%     end
    
    plotRobotModel(arm, execute_update_conf);
 
    pause(0.2);
end


%% Plot the selective prediction evolution
import gtsam.*
import gpmp2.*

figure(7);
subplot(2,1,1);
hold on;
title("Selective prediction");
axis(axis_lims);
grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');


subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

obs_factor_steps = 0:total_time_step;
obs_interp_factor_steps = setdiff(0:1/interp_multiplier:total_time_step, obs_factor_steps);

for j = 0:total_time_step
    subplot(2,1,1);
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        selective_prediction_conf = selective_prediction_case.results(j+1).atVector(symbol('x', i));
        if i<=j % executed
            selective_prediction_handle = plotArm(arm.fk_model(), selective_prediction_conf, 'm', 2);
        else % plan
            selective_prediction_handle = plotArm(arm.fk_model(), selective_prediction_conf, 'b', 2);
        end
    end
    
    
    subplot(2,1,2); 
    factor_times_updated = selective_prediction_case.update_timings.factors_steps_updated{j+1};
    
    if ~isempty(factor_times_updated)
        all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                        + [0:interp_multiplier-1] ,1, [])/interp_multiplier;
                            
        updated_interps = setdiff(all_updated_factors, obs_factor_steps);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);
    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_steps, ones(size(obs_interp_factor_steps)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_steps, ones(size(obs_factor_steps)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.5);
end

%% Plot the selective prediction evolution
import gtsam.*
import gpmp2.*

figure(8);
subplot(2,1,1);
hold on;
title("Full Prediction");
axis(axis_lims);
grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');


subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

obs_factor_steps = 0:total_time_step;
obs_interp_factor_steps = setdiff(0:1/interp_multiplier:total_time_step, obs_factor_steps);

for j = 0:total_time_step
    subplot(2,1,1);
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        prediction_conf = prediction_case.results(j+1).atVector(symbol('x', i));
        if i<=j % executed
            prediction_handle = plotArm(arm.fk_model(), prediction_conf, 'm', 2);
        else % plan
            prediction_handle = plotArm(arm.fk_model(), prediction_conf, 'b', 2);
        end
    end
    
    
    subplot(2,1,2); 
    factor_times_updated = prediction_case.update_timings.factors_steps_updated{j+1};
    
    if ~isempty(factor_times_updated)
        all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                        + [0:interp_multiplier-1] ,1, [])/interp_multiplier;
                            
        updated_interps = setdiff(all_updated_factors, obs_factor_steps);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);
    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_steps, ones(size(obs_interp_factor_steps)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_steps, ones(size(obs_factor_steps)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.5);
end

%% Plot the collision_t_update evolution
import gtsam.*
import gpmp2.*

figure(9);
hold on;
axis(axis_lims);
grid on; view(3);
xlabel('x'); ylabel('y'); zlabel('z');
for j = 0:total_time_step
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        collision_t_update_conf =collision_t_update_case.results(j+1).atVector(symbol('x', i));
        if i<=j % executed
            collision_t_update_handle = plotArm(arm.fk_model(), collision_t_update_conf, 'c', 2);
        else % plan
            collision_t_update_handle = plotArm(arm.fk_model(), collision_t_update_conf, 'b', 2);
        end
    end
    pause(0.2);
end

%%

obs_error = 0;
for j = 1:total_time_step+1
        ind = full_knowledge_case.obs_fact_indices(j);
        conf = full_knowledge_case.result.atVector(gtsam.symbol('x', j-1));
        fact = full_knowledge_case.graph.at(ind);
        
%         disp("step: " + num2str(j-1));
        disp(fact.evaluateError(conf));
%         obs_error = obs_error + sum(fact.evaluateError(conf));

end
% disp(" Error: " + num2str(obs_error));


%% Compare execute update with reinitialisation
import gtsam.*
import gpmp2.*

figure(10);
hold on;
axis(axis_lims);
grid on; view(3);
title("Execute and Update Comparison");
xlabel('x'); ylabel('y'); zlabel('z');

for j = 0:total_time_step

    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = j:total_time_step
        key_pos = symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                key_pos, arm, datasets(i+1).sdf, cost_sigma, epsilon_dist);

    
        execute_update_conf = execute_update_case.results(j+1).atVector(key_pos);
        execute_update_reinit_conf = pruning_case.results(j+1).atVector(key_pos);

        execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'b', 2);
        execute_update_reinit_handle = plotArm(arm.fk_model(), execute_update_reinit_conf, 'r', 2);

        
    end
    
    pause(0.2);
end




