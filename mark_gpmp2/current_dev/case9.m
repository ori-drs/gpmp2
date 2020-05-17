function fast_prediction_manual_case = case9(datasets, init_values, problem_setup)
%CASE2 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*

    env_size = size(datasets(1).map, 1);
    dil_env = loadPredefinedMovingEnvironment("DilatedMovingObjects", env_size, datasets(1).cell_size);
    no_stat_env = loadPredefinedMovingEnvironment("MovingReplannerNoStatic", env_size, datasets(1).cell_size);
    lab_env = loadPredefinedMovingEnvironment("Lab", env_size, datasets(1).cell_size);
    normal_env = loadPredefinedMovingEnvironment("MovingReplanner", env_size, datasets(1).cell_size);

    predicted_sdfs = cell(1,problem_setup.total_time_step+1);

    for i = 0:problem_setup.total_time_step
        t = i*problem_setup.delta_t;
        dil_dataset = dil_env.queryEnv(t);
        no_stat_dataset = no_stat_env.queryEnv(t);
        lab_dataset = lab_env.queryEnv(t);

        inds = permute(dil_dataset.map, [2,1,3]) == 1;

        new_sdf_patch = min(lab_dataset.field, no_stat_dataset.field);

        fused_field = lab_dataset.field;
        fused_field(inds) = new_sdf_patch(inds);

        predicted_sdfs{i+1} = gpmp2.SignedDistanceField(datasets(1).origin_point3, ...
                                    datasets(1).cell_size, ...
                                    size(fused_field, 1), ...
                                    size(fused_field, 2), ...
                                    size(fused_field, 3));
                                
        for z = 1:size(fused_field, 3)
            predicted_sdfs{i+1}.initFieldData(z-1, fused_field(:,:,z)');
        end
    end
    
%     object_predictor = objectTrackerPredictor(size(datasets(1).map), datasets(1).static_map,...
%                                                 problem_setup.epsilon_dist+0.05, datasets(1).cell_size, datasets(1).origin_point3);
%                      
%     predicted_sdfs = cell(1,problem_setup.total_time_step+1);
%     object_predictor.update(0, datasets(1).map);
%     object_predictor.update(0.5, datasets((0.5/problem_setup.delta_t) + 1).map); 
%     for i = 0:problem_setup.total_time_step
%         field = object_predictor.predict_object_locations(i*problem_setup.delta_t); % predict current
%         predicted_sdfs{i+1} = gpmp2.SignedDistanceField(datasets(1).origin_point3, ...
%                                     datasets(1).cell_size, ...
%                                     size(field, 1), ...
%                                     size(field, 2), ...
%                                     size(field, 3));
%                                 
%         for z = 1:size(field, 3)
%             predicted_sdfs{i+1}.initFieldData(z-1, field(:,:,z)');
%         end
%         
%     end

    
        
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
                                            predicted_sdfs{i+1}, ...
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
                        problem_setup.arm, predicted_sdfs{i+1}, ...
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
    
    fast_prediction_manual_case.final_result = result;
    fast_prediction_manual_case.graph_build_t = graph_build_t;
    fast_prediction_manual_case.graph_optimize_t = graph_optimize_t;
    fast_prediction_manual_case.graph = graph;
    fast_prediction_manual_case.all_obs_fact_indices = all_obs_fact_indices;
    fast_prediction_manual_case.obs_fact_indices = obs_fact_indices;
    fast_prediction_manual_case.iteration_costs = costs;
end

