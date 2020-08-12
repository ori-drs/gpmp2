function experiment_results = runHSRExperiment(env_size, res, start_conf, end_conf, speed)
%runHSRExperiment Summary of this function goes here
%   Detailed explanation goes here
% @author Mark Finean 
% @date July 10, 2020

    import gtsam.*
    import gpmp2.*
    
    % Setup
    total_time_sec = 3.0;
    delta_t = 0.1;
    interp_multiplier = 1;
    cost_sigma = 0.05;
    epsilon_dist = 0.3;   
    limit_v = false;
    limit_x = false;
    base_pos = [0, 0, 0.4];

    env = loadHSREnv(env_size, res, speed);

    % Planner settings
    total_time_step = round(total_time_sec/delta_t);

    problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v, base_pos);

    % get datasets
    datasets = [];

    for i = 0:total_time_step
        t = i *  delta_t;
        dataset = env.queryEnv(t);
        datasets = [datasets, dataset];
    end   
    
    %% Solve for each scenario
% 
    init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    
    eval_gp_graph = evaluationCaseNoObs(problem_setup);
    eval_obs_graph = evaluationCaseObs(datasets, problem_setup);
     
    static_case = staticUpdateCase(datasets, init_values, problem_setup);

    full_knowledge_case = fullKnowledgeUpdateCase(datasets, init_values, problem_setup);

    execute_update_case = updateCase(datasets, init_values, problem_setup);

    %% Use the full knowledge scenario to evaluate all scenarios with cost
    static_case.gp_cost = eval_gp_graph.error(static_case.final_result);
    full_knowledge_case.gp_cost = eval_gp_graph.error(full_knowledge_case.final_result);
    execute_update_case.gp_cost = eval_gp_graph.error(execute_update_case.final_result);
    static_case.obs_cost = eval_obs_graph.error(static_case.final_result);
    full_knowledge_case.obs_cost = eval_obs_graph.error(full_knowledge_case.final_result);
    execute_update_case.obs_cost = eval_obs_graph.error(execute_update_case.final_result);
    
    %% Check for collisions
 
    static_case.num_collisions = 0;
    execute_update_case.num_collisions = 0;
    full_knowledge_case.num_collisions = 0;

    for i = 0:problem_setup.total_time_step
        key_pos = gtsam.symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                     key_pos, problem_setup.arm, datasets(i+1).sdf, ...
                     cost_sigma, epsilon_dist);


        static_conf = static_case.final_result.atVector(key_pos);
        update_conf = execute_update_case.final_result.atVector(key_pos);
        full_knowledge_conf = full_knowledge_case.final_result.atVector(key_pos);
        
        if any(obs_factor.spheresInCollision(static_conf))
            static_case.num_collisions = static_case.num_collisions + 1;
        end
        
        if any(obs_factor.spheresInCollision(update_conf))
            execute_update_case.num_collisions = execute_update_case.num_collisions + 1;
        end
        
        if any(obs_factor.spheresInCollision(full_knowledge_conf))
            full_knowledge_case.num_collisions = full_knowledge_case.num_collisions + 1;
        end
        
    end

    
%%

    experiment_results.static_case = static_case;
    experiment_results.full_knowledge_case = full_knowledge_case;
    experiment_results.execute_update_case = execute_update_case;    
  
%%
%     
%     
%     clear_frames = false;
%     plot_env = true;
%     lab_axis_lims = [-1 2 -1 2 -1 2];
%     [X, Y, Z] = getEnvironmentMesh(datasets(1));
%     figure(2); hold on; cla;
%     set(gcf,'Position',[1350 500 1200 1400]);
%     axis(lab_axis_lims); 
%     grid on; view(3);
%     xlabel('x'); ylabel('y'); zlabel('z');
% 
%     traj = full_knowledge_case.final_result;
%     for i = 0:problem_setup.total_time_step
%         key_pos = gtsam.symbol('x', i);
%         conf = traj.atVector(key_pos);
%         obs_factor = gpmp2.ObstacleSDFFactorArm(...
%             key_pos, problem_setup.arm, datasets(i+1).sdf, problem_setup.cost_sigma, ...
%             problem_setup.epsilon_dist);
% 
%         if clear_frames
%             cla;
%         end
% 
%         if plot_env
%             if i > 0
%                 delete(h1);
%             end
%             h1 = plot3DEnvironment(datasets(i+1), X, Y, Z);
%         end
% 
% %         if any(obs_factor.spheresInCollision(conf))
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'r', 2);
% %         else
% %             static_handle = gpmp2.plotArm(problem_setup.arm.fk_model(), conf, 'b', 2);
% %         end
%         if any(obs_factor.spheresInCollision(conf))
%             disp('collision');
%         end
%         static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);
% 
%         
%         pause(0.5);
%     end

end

