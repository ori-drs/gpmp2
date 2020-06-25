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
    cost_sigma = 0.05;
%     cost_sigma = 0.2;
    epsilon_dist = 0.2;    
    limit_v = false;
    limit_x = false;
    
    env = loadPaperEnvs(env_num, env_size, res, speed);

    
    % Planner settings
    total_time_step = round(total_time_sec/delta_t);

    problem_setup = paperGetProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
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
    
    eval_case = fullKnowledgeCase(datasets, init_values, problem_setup);
    
%     disp('Static sdf');
    static_case = staticUpdateCase(datasets, init_values, problem_setup);

%     disp('Full knowledge sdf');
    full_knowledge_case = fullKnowledgeUpdateCase(datasets, init_values, problem_setup);

%     disp('Execute and update sdf');
    execute_update_case = updateCase(datasets, init_values, problem_setup);

%     disp('Refined static with full');
%     refined_case = fullKnowledgeUpdateCase(datasets, static_case.final_result, problem_setup);

    %% Use the full knowledge scenario to evaluate all scenarios with cost
    static_case.actual_cost = eval_case.graph.error(static_case.final_result);
    full_knowledge_case.actual_cost = eval_case.graph.error(full_knowledge_case.final_result);
    execute_update_case.actual_cost = eval_case.graph.error(execute_update_case.final_result);
%     refined_case.actual_cost = eval_case.graph.error(refined_case.final_result);
    
    %% Check for collisions
 
    static_case.num_collisions = 0;
    execute_update_case.num_collisions = 0;
    full_knowledge_case.num_collisions = 0;
%     refined_case.num_collisions = 0;
    for i = 0:problem_setup.total_time_step
        key_pos = gtsam.symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                     key_pos, problem_setup.arm, datasets(i+1).sdf, ...
                     cost_sigma, epsilon_dist);


        static_conf = static_case.final_result.atVector(key_pos);
        update_conf = execute_update_case.final_result.atVector(key_pos);
        full_knowledge_conf = full_knowledge_case.final_result.atVector(key_pos);
%         refined_conf = refined_case.final_result.atVector(key_pos);
        
        if any(obs_factor.spheresInCollision(static_conf))
            static_case.num_collisions = static_case.num_collisions + 1;
        end
        
        if any(obs_factor.spheresInCollision(update_conf))
            execute_update_case.num_collisions = execute_update_case.num_collisions + 1;
        end
        
        if any(obs_factor.spheresInCollision(full_knowledge_conf))
            full_knowledge_case.num_collisions = full_knowledge_case.num_collisions + 1;
        end
        
%         if any(obs_factor.spheresInCollision(refined_conf))
%             refined_case.num_collisions = refined_case.num_collisions + 1;
%         end
    end

    
    %%
%     clear full_knowledge_case.graph;
    
    experiment_results.static_case = static_case;
    experiment_results.full_knowledge_case = full_knowledge_case;
    experiment_results.execute_update_case = execute_update_case;    

    if full_info
%     experiment_results.refined_case = refined_case;    
        experiment_results.datasets = datasets; 
        experiment_results.problem_setup = problem_setup;
    end
end

