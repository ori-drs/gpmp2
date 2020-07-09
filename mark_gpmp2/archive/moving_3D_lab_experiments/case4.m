function selective_prediction_case = case4(datasets, init_values, problem_setup, useStaticMap)
%CASE3 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*
    
    arm_model = problem_setup.arm.fk_model();
    origin = [datasets(1).origin_x, ...
            datasets(1).origin_y, ...
            datasets(1).origin_z];
    cell_size = datasets(1).cell_size;
    origin_point3 = datasets(1).origin_point3;
    workspace_size = size(datasets(1).map);
    rows = datasets(1).rows;
    if useStaticMap
        object_predictor = objectTrackerPredictor(workspace_size, datasets(1).static_map);
    else
        object_predictor = objectTrackerPredictor(workspace_size);
    end
    
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
                                            datasets(1).sdf, ...
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
                        problem_setup.arm, datasets(1).sdf, ...
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
%     update_timings.factors_updated = cell(1, problem_setup.total_time_step+1);
    update_timings.factors_steps_updated = cell(1, problem_setup.total_time_step+1);

    predicted_sdfs = cell(problem_setup.total_time_step);
    for i = 0:problem_setup.total_time_step
        disp("Case4: Execute and selectively predict sdf... step: " + num2str(i));

        object_predictor.update(i*problem_setup.delta_t, datasets(i+1).map);
        
        if i > 0
            % Execute (get next position and add prior to graph)
            pose_key = gtsam.symbol('x', i);    
            vel_key = gtsam.symbol('v', i);    
            curr_conf = result.atVector(pose_key);
            curr_vel = result.atVector(vel_key);

            % fix conf and vel current position
            graph.add(PriorFactorVector(pose_key, curr_conf, problem_setup.pose_fix_model));
            graph.add(PriorFactorVector(vel_key, curr_vel, problem_setup.vel_fix_model));
            
            tic;
            % Update current and all future factors based on prediction
            num_factors_updated = 0;
            
            % Time to collision matrix
            D_change = (datasets(i+1).field - datasets(i).field)/problem_setup.delta_t;
            inds_ignore = D_change>0.99*min(min(min(D_change))); 
            time_to_collision = -datasets(i+1).field./D_change;
            time_to_collision(time_to_collision<0)=1000;
            time_to_collision(time_to_collision<0)=10;
            time_to_collision(time_to_collision>10)=10;
           
            % For each variable/factor set
            for j = i:problem_setup.total_time_step
                
                % Query if it could be in collision
                query_conf = result.atVector(gtsam.symbol('x', j));
                joint_positions = arm_model.forwardKinematicsPosition(query_conf)'; % [y,x,z]
                joint_positions(:, [1 2]) = joint_positions(:, [2 1]);
                joint_coords = positionToCoord(joint_positions, origin, rows, cell_size);
                
                % Swap x and y as needed to query the field correctly
%                 joint_coords(:, [1 2]) = joint_coords(:, [2 1]);
                
                query_inds = sub2ind(workspace_size,...
                                    joint_coords(:,1)', ...
                                    joint_coords(:,2)', ...
                                    joint_coords(:,3)');
                                                
                % If the collision time occurs within 2s of the joint being there, update                
                if abs((j-i) - min(time_to_collision(query_inds))) < 2
%                 if true
                    % Predict the sdf at this point
%                     disp('Map');
%                     tic;
                    predicted_map = object_predictor.predict(j*problem_setup.delta_t);
%                     toc;
%                     disp('SDF');
%                     tic;                    
                    predicted_sdfs{j} = mapToSdf(predicted_map, origin_point3, cell_size);
%                     toc;
                    
                    % Update factors
                    for k = 1:numel(all_obs_fact_indices{j+1})              
                        ind = all_obs_fact_indices{j+1}(k);
                        new_fact = graph.at(ind).getSDFModFactor(predicted_sdfs{j});            
                        graph.replace(ind, new_fact);
                    end
                    num_factors_updated = num_factors_updated + numel(all_obs_fact_indices{j+1});
%                     update_timings.factors_updated{i+1} = vertcat(update_timings.factors_updated{i+1},all_obs_fact_indices{j+1});
                    update_timings.factors_steps_updated{i+1} = vertcat(update_timings.factors_steps_updated{i+1},j);

                end
                
            end
            update_timings.factors_t(i+1) = toc;
            update_timings.num_factors(i+1) = num_factors_updated;
        end
    
        % Optimize and store result
        optimizer = GaussNewtonOptimizer(graph, result, parameters);
        
        tic;
        result = optimizer.optimize();
        update_timings.optimize_t(i+1) = toc;

        results = [results, result];

    end
    
    selective_prediction_case.final_result = result;
    selective_prediction_case.results = results;
    selective_prediction_case.graph_build_t = graph_build_t;
    selective_prediction_case.update_timings = update_timings;
    selective_prediction_case.all_obs_fact_indices = all_obs_fact_indices;
    selective_prediction_case.obs_fact_indices = obs_fact_indices;
    
end


function swapped_coord = positionToCoord(positions, origin, rows, cell_size)
    positions(:,2) = -positions(:,2);
    origin(:,2) = - origin(:,2);

    coord = round((positions - origin)/cell_size) + [1, rows ,1];
    swapped_coord = zeros(size(coord));
    swapped_coord(:,1) = coord(:,2);
    swapped_coord(:,2) = coord(:,1);
    swapped_coord(:,3) = coord(:,3);
end

function sdf = mapToSdf(map, origin_point3, cell_size)
    % signed distance field
    field  = gpmp2.signedDistanceField3D(permute(map, [2 1 3]), ...
                                                    cell_size);            

    % init sdf
    sdf = gpmp2.SignedDistanceField(origin_point3, ...
                                        cell_size, ...
                                        size(field, 1), ...
                                        size(field, 2), ...
                                        size(field, 3));
    for z = 1:size(field, 3)
        sdf.initFieldData(z-1, field(:,:,z)');
    end

end     