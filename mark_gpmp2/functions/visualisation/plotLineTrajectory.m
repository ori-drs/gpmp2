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