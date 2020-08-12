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
use_all_straight_initialisations = false;
% 
% v_or_t_end = true;
% 
% % v_or_t_end_value = [0,0, 0];
% v_or_t_end_value = [0,-0.16, 0];
% starting_pos = [0.40, 0.4, 0.4];
% % v_or_t_end_value = [0,0, 0];
% % starting_pos = [0.40, 0.0, 0.0];
% obs_size = [0.2, 0.2, 0.2];
% 
% %% Create the environment
% env = movingEnvironment3D();
% env.add_object(0,...
%                 v_or_t_end, ...
%                 v_or_t_end_value, ...
%                 starting_pos, ...
%                 obs_size);
env = loadPredefinedMovingEnvironment('MovingBlock');   
dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);


use_trustregion_opt = false;

%% Problem setup
up_conf = [0,0,0,0,0,0,0]';
forward_conf = [0,1.57,0,0,0,0,0]';

mid_point_conf = [0,1.57/2,0,0,0,0,0]';


start_conf = up_conf;
end_conf = forward_conf;
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
total_time_sec = 5.0;
delta_t = 0.25;
total_time_step = round(total_time_sec/delta_t);
interp_multiplier = 10;
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

%% get datasets

disp('Getting all the datasets');

datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   

%% build graphs

disp('Case1: Static sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
static_case = case1(start_sdf, init_values, problem_setup);

disp('Case2: Full knowledge sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
full_knowledge_case = case2(datasets, init_values, problem_setup);

disp('Case3: Execute and update sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
execute_update_case = case3(datasets, init_values, problem_setup);

disp('Case4: Execute and selectively predict sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
selective_prediction_case = case4(datasets, init_values, problem_setup, true);

disp('Case5: Execute and selectively update sdf');
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
collision_t_update_case = case5(datasets, init_values, problem_setup);

% THIS IS WAY TOO SLOW
% disp('Case6: Execute and predict sdf');
% init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
% prediction_case = case6(datasets, init_values, problem_setup);

%% Plot the comparison animation
import gtsam.*
import gpmp2.*

figure(3);
hold on;
% gpmp2.set3DPlotRange(dataset);
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);
for i = 0:total_time_step
    static_conf = static_case.result.atVector(symbol('x', i));
    full_conf = full_knowledge_case.result.atVector(symbol('x', i));
    execute_update_conf = execute_update_case.final_result.atVector(symbol('x', i));
    selective_prediction_conf =selective_prediction_case.final_result.atVector(symbol('x', i));
    collision_t_update_conf =collision_t_update_case.final_result.atVector(symbol('x', i));
%     cla;
%     if i > 0
%         delete(h1);
%     end
%     h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
    static_handle = plotArm(arm.fk_model(), static_conf, 'b', 2);
    full_handle = plotArm(arm.fk_model(), full_conf, 'r', 2);
    execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'g', 2);
    selective_prediction_handle = plotArm(arm.fk_model(), selective_prediction_conf, 'm', 2);
    collision_t_update_handle = plotArm(arm.fk_model(), collision_t_update_conf, 'c', 2);
    
%     plotRobotModel(arm, conf)
    legend([static_handle(1), full_handle(1), ... 
        execute_update_handle(1), selective_prediction_handle(1)], ...
        {"No SDF update", "Full knowledge", ...
        "Update each step", "Selective prediction"},...
        'Location','southoutside',...
        'NumColumns', 3, ...
        'FontSize', 16);
    pause(0.4);
end


%% Plot the full knowledge
import gtsam.*
import gpmp2.*

figure(8);
hold on;
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);
title("Full knowledge");
xlabel('x'); ylabel('y'); zlabel('z');

for i = 0:total_time_step
    cla;
    h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);

    full_knowledge_conf = full_knowledge_case.result.atVector(symbol('x', i));
    if i<=j % executed
        execute_update_handle = plotArm(arm.fk_model(), full_knowledge_conf, 'g', 2);
    else % plan
        execute_update_handle = plotArm(arm.fk_model(), full_knowledge_conf, 'b', 2);
    end
    pause(0.2);

end


%% Plot the execute update evolution
import gtsam.*
import gpmp2.*

figure(4);
subplot(2,1,1);
hold on;
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);
title("Execute and Update");
xlabel('x'); ylabel('y'); zlabel('z');

subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

obs_factor_times = 0:total_time_step;
obs_interp_factor_times = setdiff(0:1/interp_multiplier:total_time_step, obs_factor_times);



for j = 0:total_time_step
    subplot(2,1,1);
    cla;
    h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);
    
    for i = 0:total_time_step
        execute_update_conf = execute_update_case.results(j+1).atVector(symbol('x', i));
        if i<=j % executed
            execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'g', 2);
        else % plan
            execute_update_handle = plotArm(arm.fk_model(), execute_update_conf, 'b', 2);
        end
    end
    
    subplot(2,1,2); 
    factor_times_updated = execute_update_case.update_timings.factors_steps_updated{j+1};
    
    if ~isempty(factor_times_updated)
        all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                        + [0:interp_multiplier-1] ,1, [])/interp_multiplier;
                                    
        updated_interps = setdiff(all_updated_factors, obs_factor_times);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);

    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_times, ones(size(obs_interp_factor_times)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_times, ones(size(obs_factor_times)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.2);
end

%% Plot the selective prediction evolution
import gtsam.*
import gpmp2.*

figure(5);
subplot(2,1,1);
hold on;
title("Selective prediction");
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);
xlabel('x'); ylabel('y'); zlabel('z');


subplot(2,1,2); hold on;
title("Factor Graph");
xlabel('Time'); xlim([-1,total_time_step + 1]);
ylim([0.98,1.02]); pbaspect([10 1 1]);
set(gca,'ytick',[]) ;set(gca,'yticklabel',[])
set(gca,'Xtick',0:total_time_step)
sz1 = 500; sz2 = 25;

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
                                    
        updated_interps = setdiff(all_updated_factors, obs_factor_times);
        updated_supports = setdiff(all_updated_factors, updated_interps);    
        updated_interps = updated_interps(updated_interps<=total_time_step);
    else
        updated_interps = []; updated_supports = [];
    end

    scatter(obs_interp_factor_times, ones(size(obs_interp_factor_times)), sz2, 'b', 'filled');
    scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
    scatter(obs_factor_times, ones(size(obs_factor_times)), sz1, 'b', 'filled');
    scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');
    
    pause(0.2);
end

%% Plot the collision_t_update evolution
import gtsam.*
import gpmp2.*

figure(7);
hold on;
axis([-0.5 1 -0.5 0.5 -0.2 1]);
grid on; view(45,45);
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


% 
% figure(4);
% hold on;
% axis([0 1 -0.5 0.5 0 1]);
% grid on, view(3)
% conf = result.atVector(gtsam.symbol('x', 10));
% plot3DEnvironment(datasets(11), X, Y, Z)
% gpmp2.plotArm(arm.fk_modelstarting_pos(), conf, 'b', 2)
% 
% mid_factor = full_knowledge_graph.at(102);
% 
% for i = 1:numel(obs_fact_indices)
%     ind = obs_fact_indices(i);
%     sum(full_knowledge_graph.at(ind).evaluateError(conf))
% end




% 
% % For each time step
% for i = 0:total_time_step
%     %for each obstacle factor
% %     full_knowledge_graph.error(full_result)
%     obs_error = 0;
%     conf = full_result.atVector(gtsam.symbol('x', i));
% 
%     for j = 1:total_time_step+1
%         ind = obs_fact_indices(j);
%         fact = full_knowledge_graph.at(ind);
%         obs_error = obs_error + fact.error(full_result);
% %         obs_error = obs_error + sum(fact.evaluateError(conf));
% %         obs_error = obs_error + fact.error(static_result);
% %         fact.error(static_result)
%     end
%     disp("Time: " + num2str(i) + " Error: " + num2str(obs_error));
% end
% 
% 


%%
% 
% mid_point_conf = [0,1.57/2,0,0,0,0,0]';
% 
% obs_error = 0;
% for j = 1:total_time_step+1
%         ind = full_knowledge_case.  obs_fact_indices(j);
%         fact = full_knowledge_graph.at(ind);
%         obs_error = obs_error + sum(fact.evaluateError(mid_point_conf));
% end
% disp(" Error: " + num2str(obs_error));
%     
% figure(5);
% hold on;
% axis([0 1 -0.5 0.5 0 1]);
% grid on, view(3)
% plot3DEnvironment(datasets(11), X, Y, Z)
% plotArm(arm.fk_model(), mid_point_conf, 'r', 2)
% 
