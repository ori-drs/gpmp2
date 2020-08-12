function graph = evaluationCaseNoObs(problem_setup)
%CASE2 Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*

    %% TODO - Remove this. This should be robot independent
    joint_limit_vec_down = [-2.6, -2, -2.8, -0.9, -4.8, -1.6, -2.2]';
    joint_limit_vec_up = [2.6, 2.0, 2.8, 3.1, 1.3, 1.6, 2.2]';
    joint_limit_thresh = 0.001 * ones(7,1);
    joint_limit_model = noiseModel.Isotropic.Sigma(7, 0.001);

    % joint velocity limit param
    joint_vel_limit_vec = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]';
    joint_vel_limit_thresh = 0.01 * ones(7,1);
    joint_vel_limit_model = noiseModel.Isotropic.Sigma(7, 0.1);
    
    %%
    
    graph = NonlinearFactorGraph;

    for i = 0:problem_setup.total_time_step
        pose_key = gtsam.symbol('x', i);
        vel_key = gtsam.symbol('v', i);

        % start and end
        if i == 0
          graph.add(PriorFactorVector(pose_key, problem_setup.start_conf, problem_setup.pose_fix_model));
          graph.add(PriorFactorVector(vel_key, problem_setup.start_vel, problem_setup.vel_fix_model));
        elseif i == problem_setup.total_time_step
          graph.add(PriorFactorVector(pose_key, problem_setup.end_conf, problem_setup.pose_fix_model));
          graph.add(PriorFactorVector(vel_key, problem_setup.end_vel, problem_setup.vel_fix_model));

        end
        
        % joint limit factor on every pose
        if problem_setup.limit_x
            graph.add(JointLimitFactorVector(pose_key, joint_limit_model, joint_limit_vec_down, ...
                joint_limit_vec_up, joint_limit_thresh));

        end

        % joint velocity limit factor on every velocity
        if problem_setup.limit_v
            graph.add(VelocityLimitFactorVector(vel_key, joint_vel_limit_model, ...
                joint_vel_limit_vec, joint_vel_limit_thresh));
        end
    
        if i > 0
            last_pose_key = gtsam.symbol('x', i-1);
            last_vel_key = gtsam.symbol('v', i-1);

          % GP factor
            graph.add(GaussianProcessPriorLinear(last_pose_key, ...
                                                    last_vel_key, ...
                                                    pose_key, ...
                                                    vel_key, ...
                                                    problem_setup.delta_t, ...
                                                    problem_setup.Qc_model));
        end

    end

end

