
function plotRobotModelTrajectory(traj, datasets, problem, ax_lims, pause_time)

    [X, Y, Z] = getEnvironmentMesh(datasets(1));

%     figure(f_num); hold on;
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