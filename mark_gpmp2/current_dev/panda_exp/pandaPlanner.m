 classdef pandaPlanner < handle
    %OBJECTTRACKERPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       graph
       all_obs_fact_indices
       problem_setup
       parameters
    end
    
    methods
        
        function obj = pandaPlanner(initial_sdf, problem_setup)
            
            obj.problem_setup = problem_setup;
            obj.parameters = gtsam.GaussNewtonParams;
            obj.parameters.setVerbosity('NONE');
            
            
            joint_limit_vec_down = [-2.6, -2, -2.8, -0.9, -4.8, -1.6, -2.2]';
            joint_limit_vec_up = [2.6, 2.0, 2.8, 3.1, 1.3, 1.6, 2.2]';
            joint_limit_thresh = 0.001 * ones(7,1);
            joint_limit_model = gtsam.noiseModel.Isotropic.Sigma(7, 0.001);

            % joint velocity limit param
            joint_vel_limit_vec = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]';
            joint_vel_limit_thresh = 0.01 * ones(7,1);
            joint_vel_limit_model = gtsam.noiseModel.Isotropic.Sigma(7, 0.1);

            obj.graph = gtsam.NonlinearFactorGraph;

%             obs_fact_indices = zeros(1, problem_setup.total_time_step+1);
%             obj.gp_fact_indices = zeros(1, problem_setup.total_time_step+1);
            obj.all_obs_fact_indices = cell(1, problem_setup.total_time_step+1);

            factor_ind_counter = 0;
            for i = 0:problem_setup.total_time_step
                pose_key = gtsam.symbol('x', i);
                vel_key = gtsam.symbol('v', i);

                obs_fact_indices_in_timestep = [];

                % start and end
                if i == 0
                  obj.graph.add(gtsam.PriorFactorVector(pose_key, problem_setup.start_conf, problem_setup.pose_fix_model));
                  obj.graph.add(gtsam.PriorFactorVector(vel_key, problem_setup.start_vel, problem_setup.vel_fix_model));
                  factor_ind_counter = factor_ind_counter + 2;
                elseif i == problem_setup.total_time_step
                  obj.graph.add(gtsam.PriorFactorVector(pose_key, problem_setup.end_conf, problem_setup.pose_fix_model));
                  obj.graph.add(gtsam.PriorFactorVector(vel_key, problem_setup.end_vel, problem_setup.vel_fix_model));
                  factor_ind_counter = factor_ind_counter + 2;
                end

                % joint limit factor on every pose
                if problem_setup.limit_x
                    obj.graph.add(gpmp2.JointLimitFactorVector(pose_key, joint_limit_model, joint_limit_vec_down, ...
                        joint_limit_vec_up, joint_limit_thresh));
                    factor_ind_counter = factor_ind_counter + 1;
                end

                % joint velocity limit factor on every velocity
                if problem_setup.limit_v
                    obj.graph.add(gpmp2.VelocityLimitFactorVector(vel_key, joint_vel_limit_model, ...
                        joint_vel_limit_vec, joint_vel_limit_thresh));
                    factor_ind_counter = factor_ind_counter + 1;
                end


%                 obj.obs_fact_indices(i+1) = factor_ind_counter;
                obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; factor_ind_counter];

                obj.graph.add(gpmp2.ObstacleSDFFactorArm(pose_key, ...
                                                    problem_setup.arm, ...
                                                    initial_sdf, ...
                                                    problem_setup.cost_sigma, ...
                                                    problem_setup.epsilon_dist));
                factor_ind_counter = factor_ind_counter + 1;

                if i > 0
                    last_pose_key = gtsam.symbol('x', i-1);
                    last_vel_key = gtsam.symbol('v', i-1);

                    if problem_setup.check_inter > 0
                        for j = 1:problem_setup.check_inter 
                            tau = j * (problem_setup.total_time_sec / problem_setup.total_check_step);

                            obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; factor_ind_counter];
                            obj.graph.add(gpmp2.ObstacleSDFFactorGPArm( ...
                                last_pose_key, last_vel_key, pose_key, vel_key, ...
                                problem_setup.arm, initial_sdf, ...
                                problem_setup.cost_sigma, problem_setup.epsilon_dist, ...
                                problem_setup.Qc_model, problem_setup.delta_t, tau));
                            factor_ind_counter = factor_ind_counter + 1;

                        end
                    end

%                     obj.gp_fact_indices(i+1) = factor_ind_counter;

                    obj.graph.add(gpmp2.GaussianProcessPriorLinear(last_pose_key, ...
                                                            last_vel_key, ...
                                                            pose_key, ...
                                                            vel_key, ...
                                                            problem_setup.delta_t, ...
                                                            problem_setup.Qc_model));
                    factor_ind_counter = factor_ind_counter + 1;

                end

                obj.all_obs_fact_indices{i+1} = obs_fact_indices_in_timestep;

            end

            
        end
        
        
        function update_confs(obj, time_step, curr_conf, curr_vel)

            if time_step > 0
                pose_key = gtsam.symbol('x', time_step);    
                vel_key = gtsam.symbol('v', time_step);    

                % fix conf and vel current position
                obj.graph.add(gtsam.PriorFactorVector(pose_key, curr_conf, obj.problem_setup.pose_fix_model));
                obj.graph.add(gtsam.PriorFactorVector(vel_key, curr_vel, obj.problem_setup.vel_fix_model));
            end
        end

        function update_sdf(obj, time_step, new_sdf)

            if time_step > 0
                for k = 1:numel(obj.all_obs_fact_indices{time_step+1})              
                    ind = obj.all_obs_fact_indices{time_step+1}(k);
                    new_fact = obj.graph.at(ind).getSDFModFactor(new_sdf);            
                    obj.graph.replace(ind, new_fact);
                end
            end
        end % function     
        
        function update_all_sdfs(obj, time_step, new_sdf)

            if time_step > 0
                % Update current and all future factors to what you see now
                for j = time_step:obj.problem_setup.total_time_step
                    for k = 1:numel(obj.all_obs_fact_indices{j+1})              
                        ind = obj.all_obs_fact_indices{j+1}(k);
                        new_fact = obj.graph.at(ind).getSDFModFactor(new_sdf);            
                        obj.graph.replace(ind, new_fact);
                    end
                end 
            
            end

        end 
        
        function result = optimize(obj, result)

            optimizer = gtsam.GaussNewtonOptimizer(obj.graph, result, obj.parameters);
            optimizer.optimize();
            result = optimizer.values();

        end % function
        
    end % methods
 end % class

