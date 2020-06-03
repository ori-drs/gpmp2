function experiment_results = runPaperExperiment(env_size, res, start_conf, end_conf, env_num, speed)
%PAPERRUNEXPERIMENTS Summary of this function goes here
%   Detailed explanation goes here
% @author Mark Finean 
% @date June 01, 2020

    import gtsam.*
    import gpmp2.*
    
    %% Setup

    total_time_sec = 3.0;
    delta_t = 0.1;
    interp_multiplier = 20;
    cost_sigma = 0.05;
    epsilon_dist = 0.3;    
    limit_v = true;
    limit_x = true;
    
    env = loadPaperEnvs(env_num, env_size, res, speed);

    
    %% Planner settings

    total_time_step = round(total_time_sec/delta_t);

    problem_setup = paperGetProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v);
    
    %% get datasets
    disp('Getting all the datasets');
    datasets = [];

    for i = 0:total_time_step
        t = i *  delta_t;
        dataset = env.queryEnv(t);
        datasets = [datasets, dataset];
    end   
    
    %% Solve for each scenario
    
    init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
    
    disp('Case1: Static sdf');
    static_case = staticCase(env.queryEnv(0).sdf, init_values, problem_setup);

    disp('Case2: Full knowledge sdf');
    full_knowledge_case = fullKnowledgeCase(datasets, init_values, problem_setup);

    disp('Case3: Execute and update sdf');
    execute_update_case = updateCase(datasets, init_values, problem_setup);
    
    %% Use the full knowledge scenario to evaluate all scenarios with cost
    static_case.actual_cost = full_knowledge_case.graph.error(static_case.final_result);
    full_knowledge_case.actual_cost = full_knowledge_case.graph.error(full_knowledge_case.final_result);
    execute_update_case.actual_cost = full_knowledge_case.graph.error(execute_update_case.final_result);
    
    %% Check fol collisions
    static_confs_in_collision = 0;
    update_confs_in_collision = 0;
    full_knowledge_confs_in_collision = 0;
    for i = 0:problem_setup.total_time_step
        key_pos = gtsam.symbol('x', i);
        obs_factor = gpmp2.ObstacleSDFFactorArm(...
                     key_pos, problem_setup.arm, datasets(i+1).sdf, ...
                     cost_sigma, epsilon_dist);


        static_conf = static_case.final_result.atVector(key_pos);
        update_conf = execute_update_case.final_result.atVector(key_pos);
        full_knowledge_conf = full_knowledge_case.final_result.atVector(key_pos);
        
        if any(obs_factor.spheresInCollision(static_conf))
            static_confs_in_collision = static_confs_in_collision + 1;
        end
        
        if any(obs_factor.spheresInCollision(update_conf))
            update_confs_in_collision = update_confs_in_collision + 1;
        end
        
        if any(obs_factor.spheresInCollision(full_knowledge_conf))
            full_knowledge_confs_in_collision = full_knowledge_confs_in_collision + 1;
        end
    end

    
    
    static_case.num_collisions = static_confs_in_collision;
    full_knowledge_case.num_collisions = full_knowledge_confs_in_collision;
    execute_update_case.num_collisions = update_confs_in_collision;
    
    
    %%
    clear full_knowledge_case.graph;
    
    experiment_results.static_case = static_case;
    experiment_results.full_knowledge_case = full_knowledge_case;
    experiment_results.execute_update_case = execute_update_case;    
    experiment_results.datasets = datasets; 
    experiment_results.problem_setup = problem_setup; 
end

