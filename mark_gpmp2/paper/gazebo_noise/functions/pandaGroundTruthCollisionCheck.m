function [smoothness_cost, num_collisions] = pandaGroundTruthCollisionCheck(env_size, cell_size, problem_setup, result)

    import gpmp2.*
    import gtsam.*

    % HSR trajectory parameters
    start_pos = [0.4, 2.0, 0];
    velocity = [0, -1.2, 0];

    % Environment parameters 
    origin = [-1,-1,-1];
    scene = "tables"; 
    obstacle = "hsrb";

    % Create the environment
    env = rebuttalGroundTruthEnvironment(env_size, cell_size, origin, obstacle, scene);
    env.add_table_static_scene();

    % To store the sdfs
    sdfs = cell(1, problem_setup.total_time_step + 1);
    obs_factors = cell(1, problem_setup.total_time_step + 1);
    datasets = cell(1, problem_setup.total_time_step + 1);
    
    % Calculate the ground truth SDFs
    i=1;
    for t = 0:problem_setup.delta_t:problem_setup.total_time_sec
        key_pos = gtsam.symbol('x', i);
        pos = start_pos + t*velocity;
        env.updateMap(pos);
        sdfs{i} = env.getSDF();
        datasets{i} = env.dataset;
        obs_factors{i} = gpmp2.ObstacleSDFFactorArm(key_pos, problem_setup.arm, sdfs{i}, ...
                                                    problem_setup.cost_sigma, problem_setup.epsilon_dist);
        i = i + 1;
    end

%     obstacle_graph = pandaObstacleGraph(sdfs, problem_setup);
    smoothness_graph = pandaSmoothnessGraph(problem_setup);
    smoothness_cost = smoothness_graph.error(result);

    
    num_collisions = 0;
    for i = 0:problem_setup.total_time_step
        key_pos = gtsam.symbol('x', i);

        conf = result.atVector(key_pos);

        if any(obs_factors{i+1}.spheresInCollision(conf))
            num_collisions = num_collisions + 1;
        end    
    end

    
end

% lab_axis_lims = [-1 2 -1 2 -1 2];
% [X, Y, Z] = getEnvironmentMesh(datasets{i+1});
% figure(2); hold on; cla;
% set(gcf,'Position',[1350 500 1200 1400]);
% axis(lab_axis_lims); 
% grid on; 
% view(3);
% xlabel('x'); ylabel('y'); zlabel('z');
% h1 = plot3DEnvironment(datasets{i+1}.map, X, Y, Z);
% static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);


