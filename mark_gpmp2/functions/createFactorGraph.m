function [graph,obs_graph, obs_factor_inds_and_time] = createFactorGraph(dataset, start_conf,end_conf, start_vel, end_vel,...
                                    total_time_sec, total_check_step,total_time_step, delta_t, Qc_model, check_inter, ...
                                    arm, cost_sigma, epsilon_dist, use_GP_inter, ...
                                    joint_vel_limit_model, joint_vel_limit_vec, joint_vel_limit_thresh,flag_joint_vel_limit,...
                                    pose_fix, vel_fix)
%CREATEFACTORGRAPH Summary of this function goes here
%   Detailed explanation goes here

    graph = gtsam.NonlinearFactorGraph;
    obs_graph = gtsam.NonlinearFactorGraph;

%     init_values = gtsam.Values;
    obs_factor_inds_and_time = [];
    factor_ind = 0;
    for i = 0 : total_time_step
        key_pos = gtsam.symbol('x', i);
        key_vel = gtsam.symbol('v', i);
 
%         % initialize as straight line in conf space
%         pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
%         vel = avg_vel;
%         init_values.insert(key_pos, pose);
%         init_values.insert(key_vel, vel);

        % joint velocity limit factor on every velocity
        if flag_joint_vel_limit
            graph.add(gpmp2.VelocityLimitFactorVector(key_vel, joint_vel_limit_model, ...
                joint_vel_limit_vec, joint_vel_limit_thresh));
            factor_ind = factor_ind + 1;
        end    

        % start/end priors
        if i==0
            graph.add(gtsam.PriorFactorVector(key_pos, start_conf, pose_fix));
            graph.add(gtsam.PriorFactorVector(key_vel, start_vel, vel_fix));
            factor_ind = factor_ind + 1;
        end

        % GP priors and cost factor
        if i > 0
            key_pos1 = gtsam.symbol('x', i-1);
            key_pos2 = gtsam.symbol('x', i);
            key_vel1 = gtsam.symbol('v', i-1);
            key_vel2 = gtsam.symbol('v', i);
            graph.add(gpmp2.GaussianProcessPriorLinear(key_pos1, key_vel1, ...
                key_pos2, key_vel2, delta_t, Qc_model));
            factor_ind = factor_ind + 1;

            % cost factor
            graph.add(gpmp2.ObstaclePlanarSDFFactorArm(...
                key_pos, arm, dataset.sdf, cost_sigma, epsilon_dist));
            factor_ind = factor_ind + 1;
            obs_factor_inds_and_time = [obs_factor_inds_and_time, [factor_ind; i]];
            
            obs_graph.add(gpmp2.ObstaclePlanarSDFFactorArm(...
                key_pos, arm, dataset.sdf, cost_sigma, epsilon_dist));

            % GP cost factor
            if use_GP_inter & check_inter > 0
                for j = 1:check_inter
                    tau = j * (total_time_sec / total_check_step);
                    graph.add(gpmp2.ObstaclePlanarSDFFactorGPArm( ...
                        key_pos1, key_vel1, key_pos2, key_vel2, ...
                        arm, dataset.sdf, cost_sigma, epsilon_dist, ...
                        Qc_model, delta_t, tau));
                    factor_ind = factor_ind + 1;
                    obs_factor_inds_and_time = [obs_factor_inds_and_time, [factor_ind; i]];
            
                    obs_graph.add(gpmp2.ObstaclePlanarSDFFactorGPArm( ...
                        key_pos1, key_vel1, key_pos2, key_vel2, ...
                        arm, dataset.sdf, cost_sigma, epsilon_dist, ...
                        Qc_model, delta_t, tau));
                end
            end
        end

        if i==total_time_step
            graph.add(gtsam.PriorFactorVector(key_pos, end_conf, pose_fix));
            graph.add(gtsam.PriorFactorVector(key_vel, end_vel, vel_fix));
            factor_ind = factor_ind + 1;
        end

    end
end

