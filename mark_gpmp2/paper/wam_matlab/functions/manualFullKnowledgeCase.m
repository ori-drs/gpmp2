function case_results = manualFullKnowledgeCase(datasets, init_values, problem_setup)
%CASE2 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*

    joint_limit_vec_down = [-2.6, -2, -2.8, -0.9, -4.8, -1.6, -2.2]';
    joint_limit_vec_up = [2.6, 2.0, 2.8, 3.1, 1.3, 1.6, 2.2]';
    joint_limit_thresh = 0.001 * ones(7,1);
    joint_limit_model = noiseModel.Isotropic.Sigma(7, 0.001);

    % joint velocity limit param
    joint_vel_limit_vec = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]';
    joint_vel_limit_thresh = 0.01 * ones(7,1);
    joint_vel_limit_model = noiseModel.Isotropic.Sigma(7, 0.1);

    
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
        
        % joint limit factor on every pose
        if problem_setup.limit_x
            graph.add(JointLimitFactorVector(pose_key, joint_limit_model, joint_limit_vec_down, ...
                joint_limit_vec_up, joint_limit_thresh));
            factor_ind_counter = factor_ind_counter + 1;

        end

        % joint velocity limit factor on every velocity
        if problem_setup.limit_v
            graph.add(VelocityLimitFactorVector(vel_key, joint_vel_limit_model, ...
                joint_vel_limit_vec, joint_vel_limit_thresh));
            factor_ind_counter = factor_ind_counter + 1;
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
        parameters.setVerbosity('ERROR');
        optimizer = DoglegOptimizer(graph, init_values, parameters);
    else
        parameters = GaussNewtonParams;
        parameters.setVerbosity('ERROR');
        optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
    end
    
    error = graph.error(init_values);
    costs = error;
    traj_results{1} = init_values;
    tic;
    for i= 1:100
        optimizer.iterate();
        error_change = (error - optimizer.error)/error;
        if error_change<=1e-5 || optimizer.error>error
            break
        end
        traj_results{i+1} = optimizer.values();
        error = optimizer.error;
        costs = [costs; optimizer.error]; 
    end

%     tic;
%     optimizer.optimize();
    result = optimizer.values();
    graph_optimize_t = toc;
    
    case_results.final_result = result;
    case_results.graph_build_t = graph_build_t;
    case_results.graph_optimize_t = graph_optimize_t;
    case_results.graph = graph;
    case_results.traj_results = traj_results;
    %case_results.all_obs_fact_indices = all_obs_fact_indices;
    %case_results.obs_fact_indices = obs_fact_indices;
end

