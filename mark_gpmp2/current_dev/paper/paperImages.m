
% @author Mark Finean 
% @date May 07, 2020

close all;
clear all;
clc;
    
import gpmp2.*
import gtsam.*

plot_figs = false;
fig_num = 1;

% Run the lab experiments
cases_to_run = [1,2,3];
env_size = 75;
res = 0.04;
% all_cases = runMovingLabExperiments(env_size, res, cases_to_run);
% problem_setup = all_cases.problem_setup;

lab_env = loadPredefinedMovingEnvironment('Lab', env_size, res);
lab_dataset = lab_env.queryEnv(0);

pillars_env = loadPredefinedMovingEnvironment('MovingReplannerNoStatic', env_size, res);
pillars_dataset = pillars_env.queryEnv(0);

lab_axis_lims = [-1 1.5 -1.2 1.5 -1 2];
[X, Y, Z] = getEnvironmentMesh(lab_dataset);

%% Data wanted for the paper

figure(fig_num); hold on; 
set(gcf,'Position',[1350 500 1200 1400]);
view(3);
c = colormap(autumn);
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');

plot3DEnvironment(lab_dataset, X, Y, Z+1, 1, [171, 104, 87]/255)

plot3DEnvironment(pillars_env.queryEnv(0), X, Y, Z+1, 1, [0 0.5 1])
plot3DEnvironment(pillars_env.queryEnv(0.7), X, Y, Z+1, 0.5, [0 0.5 1])
plot3DEnvironment(pillars_env.queryEnv(1.4), X, Y, Z+1, 0.2, [0 0.5 1])
axis(lab_axis_lims);
axis("equal");




















%% Plotting functions
function plotSetupProblem(all_cases)
    figure(fig_num); hold on; 
    set(gcf,'Position',[1350 500 1200 1400]);
    title('3D Environment')
    grid on;
%     view(3);
    view(-1.007130428616419e+02, 50.314206527540222);
    campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    
    gpmp2.set3DPlotRange(all_cases.datasets(1));
    xlabel('x'); ylabel('y'); zlabel('z');
    plot3DEnvironment(all_cases.datasets(1), X, Y, Z)
    gpmp2.plotArm(all_cases.problem.arm.fk_model(), ...
            problem.start_conf, 'b', 2)
    gpmp2.plotArm(all_cases.problem.arm.fk_model(), ...
            problem.end_conf, 'r', 2)

end

function plotSceneEvolution(all_cases, pause_time)
    figure(fig_num); hold on;
    set(gcf,'Position',[1350 500 1200 1400]);
    gpmp2.set3DPlotRange(all_cases.datasets(1));
    grid on, view(3);
    for t = 1:all_cases.problem_setup.total_time_step+1
        cla;
        plot3DEnvironment(all_cases.datasets(t), X, Y, Z) 
        pause(pause_time);
    end
end

function plotTrajComparison(all_data, fig_num, ax_lims, ...
                            cases_to_compare, cases_names, pause_time, remove_env)
    [X, Y, Z] = getEnvironmentMesh(all_data.datasets(1));
    c_order = ['b', 'r', 'g', 'm'];
    figure(fig_num); hold on;
    set(gcf,'Position',[1350 500 1200 1400]);
    view(-1.007130428616419e+02, 50.314206527540222);
    campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    axis(ax_lims);
    grid on; 
%     view(3);

    confs_to_compare = cell(1, length(cases_to_compare));

    for i = 0:all_data.problem_setup.total_time_step
        case_handles = [];

        if ~remove_env
            if i > 0
                delete(h1);
            end
            h1 = plot3DEnvironment(all_data.datasets(i+1), X, Y, Z);
        end
        
        for j =1:length(cases_to_compare)
            confs_to_compare{j} = cases_to_compare{j}.final_result.atVector(gtsam.symbol('x', i));
            arm_hand = gpmp2.plotArm(all_data.problem_setup.arm.fk_model(), confs_to_compare{j}, c_order(j), 2);
            case_handles = [case_handles, arm_hand(1)];
        end

        legend(case_handles, ...
            cases_names,...
            'Location','southoutside',...
            'NumColumns', 3, ...
            'FontSize', 16);
        pause(pause_time);
    end

end

function plotLineTrajectory(traj, datasets, problem, f_num, ax_lims, clear_frames, plot_env, pause_time)

    [X, Y, Z] = getEnvironmentMesh(datasets(1));

    figure(f_num); hold on;
    set(gcf,'Position',[1350 500 1200 1400]);
    axis(ax_lims);
    grid on; 
    view(-1.007130428616419e+02, 50.314206527540222);
    campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    
    title("Graph");
    xlabel('x'); ylabel('y'); zlabel('z');

    for i = 0:problem.total_time_step
        key_pos = gtsam.symbol('x', i);
        conf = traj.final_result.atVector(key_pos);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
            key_pos, problem.arm, datasets(i+1).sdf, problem.cost_sigma, ...
            problem.epsilon_dist);

        if clear_frames
            cla;
        end

        if plot_env
            if i > 0
                delete(h1);
            end
            h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
        end

        if any(obs_factor.spheresInCollision(conf))
            static_handle = gpmp2.plotArm(problem.arm.fk_model(), conf, 'r', 2);
        else
            static_handle = gpmp2.plotArm(problem.arm.fk_model(), conf, 'b', 2);
        end

%         ind = full_knowledge_case.obs_fact_indices(j);
%         fact = full_knowledge_case.graph.at(ind);
%         disp(fact.evaluateError(conf));

        pause(pause_time);
    end

end

function plotRobotModelTrajectory(traj, datasets, problem, f_num, ax_lims, pause_time)

    [X, Y, Z] = getEnvironmentMesh(datasets(1));

    figure(f_num); hold on;
    set(gcf,'Position',[1350 500 1200 1400]);
    axis(ax_lims);
    grid on; 
    view(-1.007130428616419e+02, 50.314206527540222);
    campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    
    title("Graph");
    xlabel('x'); ylabel('y'); zlabel('z');

    for i = 0:problem.total_time_step
        key_pos = gtsam.symbol('x', i);
        conf = traj.final_result.atVector(key_pos);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
            key_pos, problem.arm, datasets(i+1).sdf, problem.cost_sigma, ...
            problem.epsilon_dist);

        cla;
        h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
        gpmp2.plotRobotModel(problem.arm, conf);
        
        pause(pause_time);
    end

end

function plotTrajectoryAndStateEvolution(traj, datasets, problem, f_num, ax_lims, pause_time)

    [X, Y, Z] = getEnvironmentMesh(datasets(1));
  
    figure(f_num); hold on;
    set(gcf,'Position',[1350 500 1200 1400]);
    subplot(2, 1, 1); hold on;
    axis(ax_lims); grid on; view(3);
    view(-1.007130428616419e+02, 50.314206527540222);
    campos([-11.08977871940202,2.4122839520229,16.916622755708733]);
    title("Graph");
    xlabel('x'); ylabel('y'); zlabel('z');

    subplot(2, 1, 2); hold on;
    title("Factor Graph");
    xlabel('Time'); xlim([-1, problem.total_time_step + 1]);
    ylim([0.98, 1.02]); pbaspect([10 1 1]);
    set(gca,'ytick', []) ;set(gca,'yticklabel',[])
    set(gca,'Xtick', 0:problem.total_time_step)
    sz1 = 500; sz2 = 25;
    
    interp_multiplier = problem.check_inter + 1;
    obs_factor_steps = 0:problem.total_time_step;
    obs_interp_factor_steps = setdiff(0:1/interp_multiplier:problem.total_time_step, obs_factor_steps);

    for j = 0:problem.total_time_step

        subplot(2,1,1);
        cla;
        h1 = plot3DEnvironment(datasets(j+1), X, Y, Z);

        for i = 0:problem.total_time_step
            key_pos = gtsam.symbol('x', i);
            obs_factor = gpmp2.ObstacleSDFFactorArm(...
                         key_pos, problem.arm, datasets(i+1).sdf, ...
                         problem.cost_sigma, problem.epsilon_dist);


            conf = traj.results(j+1).atVector(key_pos);
            in_collision_bool = any(obs_factor.spheresInCollision(conf));
            if i>j
                handle = gpmp2.plotArm(problem.arm.fk_model(), conf, 'b', 2);
            elseif in_collision_bool &&  i<=j % executed and collides
                handle = gpmp2.plotArm(problem.arm.fk_model(), conf, 'r', 2);
            else  % executed 
                handle = gpmp2.plotArm(problem.arm.fk_model(), conf, 'g', 2);
            end
        end

        subplot(2,1,2); 
        factor_times_updated = traj.update_timings.factors_steps_updated{j+1};

        if ~isempty(factor_times_updated)
            all_updated_factors = reshape(repmat(factor_times_updated*interp_multiplier, 1, interp_multiplier) ... 
                                            + [0:interp_multiplier-1] ,1, [])/interp_multiplier;

            updated_interps = setdiff(all_updated_factors, obs_factor_steps);
            updated_supports = setdiff(all_updated_factors, updated_interps);    
            updated_interps = updated_interps(updated_interps<=problem.total_time_step);

        else
            updated_interps = []; updated_supports = [];
        end

        scatter(obs_interp_factor_steps, ones(size(obs_interp_factor_steps)), sz2, 'b', 'filled');
        scatter(updated_interps, ones(size(updated_interps)), sz2, 'r', 'filled');
        scatter(obs_factor_steps, ones(size(obs_factor_steps)), sz1, 'b', 'filled');
        scatter(updated_supports, ones(size(updated_supports)), sz1, 'r', 'filled');

        pause(pause_time);
    end
end


% obs_error = 0;
% for j = 1:total_time_step+1
%         ind = full_knowledge_case.obs_fact_indices(j);
%         conf = full_knowledge_case.result.atVector(gtsam.symbol('x', j-1));
%         fact = full_knowledge_case.graph.at(ind);
%         
%         disp(fact.evaluateError(conf));
% end