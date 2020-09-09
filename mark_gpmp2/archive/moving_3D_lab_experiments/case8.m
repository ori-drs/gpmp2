function pruning_reinit_case = case8(datasets, init_values, problem_setup)
%CASE3 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*
    
    tic;
    graph = NonlinearFactorGraph;

    obs_fact_indices = zeros(1, problem_setup.total_time_step+1);
    gp_fact_indices = zeros(1, problem_setup.total_time_step);
    all_obs_fact_indices = cell(1, problem_setup.total_time_step+1);
    all_gp_fact_indices = cell(1, problem_setup.total_time_step+1);
    obs_fact_indices_in_timestep = [];
    gp_fact_indices_in_timestep = [];

    factor_ind_counter = 0;
    for i = 0:problem_setup.total_time_step
        pose_key = gtsam.symbol('x', i);
        vel_key = gtsam.symbol('v', i);

        obs_fact_indices_in_timestep = [];
        gp_fact_indices_in_timestep = [];

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
                                            datasets(1).sdf, ...
                                            problem_setup.cost_sigma, ...
                                            problem_setup.epsilon_dist));
        factor_ind_counter = factor_ind_counter + 1;

        if i > 0
            last_pose_key = gtsam.symbol('x', i-1);
            last_vel_key = gtsam.symbol('v', i-1);

          % interpolated cost factor
            % GP cost factor
            interpobs_fact_indices_in_timestep = [];

            if problem_setup.check_inter > 0
                for j = 1:problem_setup.check_inter 
                    tau = j * (problem_setup.total_time_sec / problem_setup.total_check_step);

                    interpobs_fact_indices_in_timestep = [interpobs_fact_indices_in_timestep; factor_ind_counter];
                    graph.add(ObstacleSDFFactorGPArm( ...
                        last_pose_key, last_vel_key, pose_key, vel_key, ...
                        problem_setup.arm, datasets(1).sdf, ...
                        problem_setup.cost_sigma, problem_setup.epsilon_dist, ...
                        problem_setup.Qc_model, problem_setup.delta_t, tau));
                    factor_ind_counter = factor_ind_counter + 1;

                end
            end

          % GP factor
            gp_fact_indices(i) = factor_ind_counter;

            graph.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                    last_vel_key, ...
                                                    pose_key, ...
                                                    vel_key, ...
                                                    problem_setup.delta_t, ...
                                                    problem_setup.Qc_model));
            factor_ind_counter = factor_ind_counter + 1;

            all_obs_fact_indices{i} = vertcat(all_obs_fact_indices{i}, interpobs_fact_indices_in_timestep);

        end

        all_obs_fact_indices{i+1} = obs_fact_indices_in_timestep;

    end
  
    
    graph_build_t = toc;
   
    % At this point the graph is exactly the same of the static graph
    
    if problem_setup.use_trustregion_opt
        parameters = DoglegParams;
        parameters.setVerbosity('NONE');
    else
        parameters = GaussNewtonParams;
        parameters.setVerbosity('NONE');
    end
    
    results = [];
    result = init_values;

    update_timings.factors_t = zeros(1, problem_setup.total_time_step+1);
    update_timings.num_factors = zeros(1, problem_setup.total_time_step+1);
    update_timings.optimize_t = zeros(1, problem_setup.total_time_step+1);
    update_timings.factors_steps_updated = cell(1, problem_setup.total_time_step+1);
    pruning_reinit_case.gp_cost_evolution = zeros(1, problem_setup.total_time_step+1);
    pruning_reinit_case.avg_remaining_gp_cost = zeros(1, problem_setup.total_time_step+1);
    pruning_reinit_case.all_gp_factor_costs = zeros(numel(gp_fact_indices), problem_setup.total_time_step+1);
    
    for i = 0:problem_setup.total_time_step
        disp("Case8: Execute and update sdf using pruning... step: " + num2str(i));

        if i > 0
            if i==1
                graph.remove(0);
                graph.remove(1);
            end
            
            % Remove the last added factors
            if i>1
                graph.remove(graph.size()-1);
                graph.remove(graph.size()-2);
            end
            
            previous_inds_to_remove = vertcat(all_obs_fact_indices{i}, gp_fact_indices(i));
            for k = 1:numel(previous_inds_to_remove)
                graph.remove(previous_inds_to_remove(k));
            end
            
            % Execute (get next position and add prior to graph)
            pose_key = gtsam.symbol('x', i);    
            vel_key = gtsam.symbol('v', i);    
            curr_conf = result.atVector(pose_key);
            curr_vel = result.atVector(vel_key);

            % fix conf and vel current position
            graph.add(PriorFactorVector(pose_key, curr_conf, problem_setup.pose_fix_model));
            graph.add(PriorFactorVector(vel_key, curr_vel, problem_setup.vel_fix_model));

            % Update current and all future factors to what you see now
            num_factors_updated = 0;
            tic;
            for j = i:problem_setup.total_time_step
                for k = 1:numel(all_obs_fact_indices{j+1})              
                    ind = all_obs_fact_indices{j+1}(k);
                    new_fact = graph.at(ind).getSDFModFactor(datasets(i+1).sdf);            
                    graph.replace(ind, new_fact);
                end
                num_factors_updated = num_factors_updated + numel(all_obs_fact_indices{j+1});
%                 update_timings.factors_updated{i+1} = vertcat(update_timings.factors_updated{i+1},all_obs_fact_indices{j+1});
                update_timings.factors_steps_updated{i+1} = vertcat(update_timings.factors_steps_updated{i+1},j);

            end
            
            update_timings.factors_t(i+1) = toc;
            update_timings.num_factors(i+1) = num_factors_updated;

            if i < problem_setup.total_time_step && avg_remaining_gp_cost > 50
                result = gpmp2.reinitRemainderArmTrajStraightLine(result, ...
                                                                problem_setup.end_conf, ...
                                                                problem_setup.end_vel, ...
                                                                i);
            end     
            
        end
    
        % Optimize and store result
        optimizer = GaussNewtonOptimizer(graph, result, parameters);
        
        tic;
        result = optimizer.optimize();
        update_timings.optimize_t(i+1) = toc;
        results = [results, result];
        
        gp_cost = 0;
        for k = i+1:numel(gp_fact_indices)
            gp_cost = gp_cost + graph.at(gp_fact_indices(k)).error(result);   
            pruning_reinit_case.all_gp_factor_costs(k,i+1) = graph.at(gp_fact_indices(k)).error(result);   

        end
        
        avg_remaining_gp_cost = gp_cost/(numel(gp_fact_indices)-i);
        pruning_reinit_case.gp_cost_evolution(i+1) = gp_cost;
        pruning_reinit_case.avg_remaining_gp_cost(i+1) = avg_remaining_gp_cost;

    end
    
    pruning_reinit_case.final_result = result;
    pruning_reinit_case.results = results;
    pruning_reinit_case.graph_build_t = graph_build_t;
    pruning_reinit_case.update_timings = update_timings;
    pruning_reinit_case.all_obs_fact_indices = all_obs_fact_indices;
    pruning_reinit_case.all_gp_fact_indices = all_gp_fact_indices;
    pruning_reinit_case.obs_fact_indices = obs_fact_indices;
    pruning_reinit_case.gp_fact_indices = gp_fact_indices;

end
