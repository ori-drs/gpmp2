function full_knowledge_manual_case = case10(datasets, init_values, problem_setup)
%CASE2 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*
    
    tic;
    graph = NonlinearFactorGraph;

    obs_fact_indices = zeros(1, problem_setup.total_time_step+1);
    all_obs_fact_indices = cell(1, problem_setup.total_time_step+1);
    obs_fact_indices_in_timestep = [];

    factor_ind_counter = 0;
    for i = 0:problem_setup.total_time_step
        pose_key = gtsam.symbol('x', i);
        vel_key = gtsam.symbol('v', i);

        obs_fact_indices_in_timestep = [];

        % start and end
        if i == 0
          graph.add(PriorFactorVector(pose_key, problem_setup.start_conf, problem_setup.pose_fix_model));
          graph.add(PriorFactorVector(vel_key, problem_setup.start_vel, problem_setup.vel_fix_model));
          factor_ind_counter = factor_ind_counter + 2;
        elseif i == problem_setup.total_time_step
          graph.add(PriorFactorVector(pose_key, problem_setup.end_conf, problem_setup.pose_fix_model));
          graph.add(PriorFactorVector(vel_key, problem_setup.end_vel, problem_setup.vel_fix_model));
          factor_ind_counter = factor_ind_counter + 2;

        end

        % non-interpolated cost factor
        obs_fact_indices(i+1) = factor_ind_counter;
        obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; factor_ind_counter];

        graph.add(ObstacleSDFFactorArm(pose_key, ...
                                            problem_setup.arm, ...
                                            datasets(i+1).sdf, ...
                                            problem_setup.cost_sigma, ...
                                            problem_setup.epsilon_dist));
        factor_ind_counter = factor_ind_counter + 1;

        if i > 0
            last_pose_key = gtsam.symbol('x', i-1);
            last_vel_key = gtsam.symbol('v', i-1);

          % interpolated cost factor
            % GP cost factor
            if problem_setup.check_inter > 0
                for j = 1:problem_setup.check_inter 
                    tau = j * (problem_setup.total_time_sec / problem_setup.total_check_step);

                    obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; factor_ind_counter];
                    graph.add(ObstacleSDFFactorGPArm( ...
                        last_pose_key, last_vel_key, pose_key, vel_key, ...
                        problem_setup.arm, datasets(i+1).sdf, ...
                        problem_setup.cost_sigma, problem_setup.epsilon_dist, ...
                        problem_setup.Qc_model, problem_setup.delta_t, tau));
                    factor_ind_counter = factor_ind_counter + 1;

                end
            end

          % GP factor
            graph.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                    last_vel_key, ...
                                                    pose_key, ...
                                                    vel_key, ...
                                                    problem_setup.delta_t, ...
                                                    problem_setup.Qc_model));
            factor_ind_counter = factor_ind_counter + 1;

        end

        all_obs_fact_indices{i+1} = obs_fact_indices_in_timestep;

    end

    graph_build_t = toc;

    if problem_setup.use_trustregion_opt
        parameters = DoglegParams;
        parameters.setVerbosity('NONE');
        optimizer = DoglegOptimizer(graph, init_values, parameters);
    else
        parameters = GaussNewtonParams;
        parameters.setVerbosity('NONE');
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
    end
    
    error = graph.error(init_values);
    costs = error;
    tic;
    for i= 1:100
        optimizer.iterate();
        error_change = (error - optimizer.error)/error;
        if error_change<=1e-5 || optimizer.error>error
            break
        end
        error = optimizer.error;
        costs = [costs; optimizer.error]; 
    end

%     tic;
%     optimizer.optimize();
    result = optimizer.values();
    graph_optimize_t = toc;
    
    full_knowledge_manual_case.final_result = result;
    full_knowledge_manual_case.graph_build_t = graph_build_t;
    full_knowledge_manual_case.graph_optimize_t = graph_optimize_t;
    full_knowledge_manual_case.graph = graph;
    full_knowledge_manual_case.all_obs_fact_indices = all_obs_fact_indices;
    full_knowledge_manual_case.obs_fact_indices = obs_fact_indices;
    full_knowledge_manual_case.iteration_costs = costs;
end
