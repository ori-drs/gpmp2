function [graph, obs_graph, init_values] = generateFactorGraph(problem)
%GENERATEFACTORGRAPH Summary of this function goes here
%   Detailed explanation goes here
    
    import gtsam.*
    import gpmp2.*
    
    graph = gtsam.NonlinearFactorGraph;
    obs_graph = gtsam.NonlinearFactorGraph;

    init_values = gtsam.Values;

    datasets = cell(problem.total_time_step,1);

    for i = 0 : problem.total_time_step
        query_t = i * problem.delta_t;
        key_pos = symbol('x', i);
        key_vel = symbol('v', i);

        datasets{i+1} = generateMovingScene(problem.t_start_moving, problem.v_or_t_end, ...
                                            problem.v_or_t_end_value, query_t);
        % initialize as straight line in conf space
        pose = problem.start_conf * (problem.total_time_step-i)/problem.total_time_step ...
                        + problem.end_conf * i/problem.total_time_step;
        vel = problem.avg_vel;
        init_values.insert(key_pos, pose);
        init_values.insert(key_vel, vel);

        % start/end priors
        if i==0
            graph.add(PriorFactorVector(key_pos, problem.start_conf, problem.pose_fix));
            graph.add(PriorFactorVector(key_vel, problem.start_vel, problem.vel_fix));
        elseif i==problem.total_time_step
            graph.add(PriorFactorVector(key_pos, problem.end_conf, problem.pose_fix));
            graph.add(PriorFactorVector(key_vel, problem.end_vel, problem.vel_fix));
        end

        % GP priors and cost factor
        if i > 0
            key_pos1 = symbol('x', i-1);
            key_pos2 = symbol('x', i);
            key_vel1 = symbol('v', i-1);
            key_vel2 = symbol('v', i);
            graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
                key_pos2, key_vel2, problem.delta_t, problem.Qc_model));

            % cost factor
            graph.add(ObstaclePlanarSDFFactorArm(...
                key_pos, problem.arm, datasets{i+1}.sdf, problem.cost_sigma, problem.epsilon_dist));
            obs_graph.add(ObstaclePlanarSDFFactorArm(...
                key_pos, problem.arm, datasets{i+1}.sdf, problem.cost_sigma, problem.epsilon_dist));

            % GP cost factor
            if problem.use_GP_inter & problem.check_inter > 0
                for j = 1:problem.check_inter
                    tau = j * (problem.total_time_sec / problem.total_check_step);
                    graph.add(ObstaclePlanarSDFFactorGPArm( ...
                        key_pos1, key_vel1, key_pos2, key_vel2, ...
                        problem.arm, datasets{i+1}.sdf, problem.cost_sigma, problem.epsilon_dist, ...
                        problem.Qc_model, problem.delta_t, tau));
                    obs_graph.add(ObstaclePlanarSDFFactorGPArm( ...
                        key_pos1, key_vel1, key_pos2, key_vel2, ...
                        problem.arm, datasets{i+1}.sdf, problem.cost_sigma, problem.epsilon_dist, ...
                        problem.Qc_model, problem.delta_t, tau));
                end
            end
        end
    end


end

