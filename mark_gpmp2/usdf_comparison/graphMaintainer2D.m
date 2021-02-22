 classdef graphMaintainer2D < handle
    %OBJECTTRACKERPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       graph
       all_obs_fact_indices
       gp_fact_indices
       problem_setup
       parameters
       sdf
       
       joint_limit_vec_down
       joint_limit_vec_up
       joint_limit_thresh
       joint_limit_model
       joint_vel_limit_vec
       joint_vel_limit_thresh
       joint_vel_limit_model

       factor_ind_counter
    end
    
    methods
        
        function obj = graphMaintainer2D(sdf, problem_setup)
            
            obj.problem_setup = problem_setup;
%             obj.parameters = gtsam.GaussNewtonParams;
            obj.parameters = gtsam.LevenbergMarquardtParams;
            obj.parameters.setVerbosity('NONE');
            
            obj.sdf = sdf;
            
            obj.createFullGraph();
            
        end

        function obj = createFullGraph(obj)

            obj.graph = gtsam.NonlinearFactorGraph;

            obj.gp_fact_indices = zeros(1, obj.problem_setup.total_time_step+1);
            obj.all_obs_fact_indices = cell(1, obj.problem_setup.total_time_step+1);

            obj.factor_ind_counter = 0;
            for i = 0:obj.problem_setup.total_time_step
                pose_key = gtsam.symbol('x', i);
                vel_key = gtsam.symbol('v', i);

                obs_fact_indices_in_timestep = [];
    
                % start and end
                if i == 0
                  obj.graph.add(gtsam.PriorFactorPose2(pose_key, obj.problem_setup.start_pose, obj.problem_setup.pose_fix_model));
                  obj.graph.add(gtsam.PriorFactorVector(vel_key, obj.problem_setup.start_vel, obj.problem_setup.vel_fix_model));
                  obj.factor_ind_counter = obj.factor_ind_counter + 2;
                elseif i == obj.problem_setup.total_time_step
                  obj.graph.add(gtsam.PriorFactorPose2(pose_key, obj.problem_setup.end_pose, obj.problem_setup.pose_fix_model));
                  obj.graph.add(gtsam.PriorFactorVector(vel_key, obj.problem_setup.end_vel, obj.problem_setup.vel_fix_model));
                  obj.factor_ind_counter = obj.factor_ind_counter + 2;
                end

                obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; obj.factor_ind_counter];

                obj.graph.add(gpmp2.ObstaclePlanarSDFFactorPose2MobileBase(pose_key, ...
                                                    obj.problem_setup.robot, ...
                                                    obj.sdf, ...
                                                    obj.problem_setup.cost_sigma, ...
                                                    obj.problem_setup.epsilon_dist));
                obj.factor_ind_counter = obj.factor_ind_counter + 1;

                if i > 0
                    last_pose_key = gtsam.symbol('x', i-1);
                    last_vel_key = gtsam.symbol('v', i-1);

                    if obj.problem_setup.check_inter > 0
                        for j = 1:obj.problem_setup.check_inter 
                            tau = j * (obj.problem_setup.total_time_sec / obj.problem_setup.total_check_step);

                            obs_fact_indices_in_timestep = [obs_fact_indices_in_timestep; obj.factor_ind_counter];
                            obj.graph.add(gpmp2.ObstaclePlanarSDFFactorGPPose2MobileBase( ...
                                last_pose_key, last_vel_key, pose_key, vel_key, ...
                                obj.problem_setup.robot, obj.sdf, ...
                                obj.problem_setup.cost_sigma, obj.problem_setup.epsilon_dist, ...
                                obj.problem_setup.Qc_model, obj.problem_setup.delta_t, tau));
                            obj.factor_ind_counter = obj.factor_ind_counter + 1;

                        end
                    end

                    obj.gp_fact_indices(i+1) = obj.factor_ind_counter;

                    obj.graph.add(gpmp2.GaussianProcessPriorPose2(last_pose_key, ...
                                                            last_vel_key, ...
                                                            pose_key, ...
                                                            vel_key, ...
                                                            obj.problem_setup.delta_t, ...
                                                            obj.problem_setup.Qc_model));
                    obj.factor_ind_counter = obj.factor_ind_counter + 1;

                end

                obj.all_obs_fact_indices{i+1} = obs_fact_indices_in_timestep;

            end

            
        end

        function isCollision = collisionCheck(obj, conf)
            isCollision = false;

            % For each timestep
            for i = 1:obj.problem_setup.total_time_step + 1
                % For a static environment, all obstacle factors are the
                % same so just use one
                factor = obj.graph.at(obj.all_obs_fact_indices{1}(1));
                isCollision = isCollision || any(factor.spheresInCollision(conf));
            end
 
        end
        function [result, error, iterations] = optimize(obj, result)

%             optimizer = gtsam.GaussNewtonOptimizer(obj.graph, result, obj.parameters);
            optimizer = gtsam.LevenbergMarquardtOptimizer(obj.graph, result, obj.parameters);
            optimizer.optimize();
            result = optimizer.values();
            error = optimizer.error();
            iterations = optimizer.iterations();

        end % function

                
        function error = error(obj, values)
            error = obj.graph.error(values);
        end % function
        
    end % methods
 end % class

