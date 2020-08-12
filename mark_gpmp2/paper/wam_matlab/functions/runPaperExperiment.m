function experiment_results = runPaperExperiment(env_size, res, start_conf, end_conf, env_num, speed, full_info)
%PAPERRUNEXPERIMENTS Summary of this function goes here
%   Detailed explanation goes here
% @author Mark Finean 
% @date June 01, 2020

    import gtsam.*
    import gpmp2.*
    
    % Setup
    total_time_sec = 3.0;
    delta_t = 0.1;
    interp_multiplier = 20;
    cost_sigma = 0.2;
    epsilon_dist = 0.2;    
    limit_v = false;
    limit_x = false;
    
    env = loadWAMExperimentEnv(env_num, env_size, res, speed);

    
    % Planner settings
    total_time_step = round(total_time_sec/delta_t);

    problem_setup = WAMExperimentProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v);
                                            
                                     
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
%     init_values = initArmTrajRandom(start_conf, end_conf, total_time_step);
    
%     eval_case = fullKnowledgeCase(datasets, init_values, problem_setup);
    eval_gp_graph = evaluationCaseNoObs(problem_setup);
    eval_obs_graph = evaluationCaseObs(datasets, problem_setup);
    
%     disp('Static sdf');
    static_case = staticUpdateCase(datasets, init_values, problem_setup);

%     disp('Full knowledge sdf');
    full_knowledge_case = fullKnowledgeUpdateCase(datasets, init_values, problem_setup);

%     disp('Execute and update sdf');
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

    if full_info
        experiment_results.datasets = datasets; 
        experiment_results.problem_setup = problem_setup;
    end
end

