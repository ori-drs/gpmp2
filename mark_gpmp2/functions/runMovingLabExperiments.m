function all_cases = runMovingLabExperiments(env_size, res, cases_to_run)
%RUNMOVINGLABEXPERIMENTS Summary of this function goes here
%   Detailed explanation goes here
% @author Mark Finean 
% @date May 07, 2020

    import gtsam.*
    import gpmp2.*

    %% Setup
    env_name = 'MovingReplanner';
%     env = loadPredefinedMovingEnvironment(env_name, env_size, res);
    env = loadPaperEnvs(1, env_size, res, 0.6);
    dataset = env.queryEnv(0);
%     [X, Y, Z] = getEnvironmentMesh(dataset);
    use_trustregion_opt = false;

    %% Problem setup

    replanner_start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
    replanner_end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]';

    start_conf = replanner_start_conf;
    end_conf = replanner_end_conf;
    start_vel = zeros(7,1);
    end_vel = zeros(7,1);

    arm = generateArm('WAMArm');
    arm_model = arm.fk_model();

    %% Planner settings
    total_time_sec = 3.0;
%     delta_t = 0.05;
    delta_t = 0.1;
    total_time_step = round(total_time_sec/delta_t);
    interp_multiplier = 20;
    total_check_step = interp_multiplier*total_time_step;
    check_inter = total_check_step / total_time_step - 1;
    pause_time = delta_t;

    % use GP interpolation
    use_GP_inter = true;

    % GP
    Qc = 1 * eye(7);
    Qc_model = noiseModel.Gaussian.Covariance(Qc); 

    % algo settings
    cost_sigma = 0.05;
%     cost_sigma = 0.1;
    epsilon_dist = 0.2;

    % noise model
    pose_fix_sigma = 0.0001;
    vel_fix_sigma = 0.0001;

    pose_fix_model = noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
    vel_fix_model = noiseModel.Isotropic.Sigma(7, vel_fix_sigma);

    problem_setup.start_conf = start_conf;
    problem_setup.end_conf = end_conf;
    problem_setup.start_vel = start_vel;
    problem_setup.end_vel = end_vel;
    problem_setup.total_time_step = total_time_step;
    problem_setup.total_time_sec = total_time_sec;
    problem_setup.total_check_step = total_check_step;
    problem_setup.delta_t = delta_t;
    problem_setup.check_inter = check_inter;
    problem_setup.pose_fix_sigma = pose_fix_sigma;
    problem_setup.pose_fix_model = pose_fix_model;
    problem_setup.vel_fix_sigma = vel_fix_sigma;
    problem_setup.vel_fix_model = vel_fix_model;
    problem_setup.cost_sigma = cost_sigma;
    problem_setup.epsilon_dist = epsilon_dist;
    problem_setup.arm = arm;
    problem_setup.Qc_model = Qc_model;
    problem_setup.use_trustregion_opt = use_trustregion_opt;

    % initial values by batch
    init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

    start_sdf = env.queryEnv(0).sdf;

    %% get datasets
    disp('Getting all the datasets');
    datasets = [];

    for i = 0:total_time_step
        t = i *  delta_t;
        dataset = env.queryEnv(t);
        datasets = [datasets, dataset];
    end   

    all_cases.problem_setup = problem_setup;
    all_cases.datasets = datasets;
    
    %% build graphs
    
    for i = cases_to_run
    
        switch i
            case 1
                disp('Case1: Static sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                static_case = case1(start_sdf, init_values, problem_setup);
                all_cases.static_case = static_case;
                
            case 2
                disp('Case2: Full knowledge sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                full_knowledge_case = case2(datasets, init_values, problem_setup);
                all_cases.full_knowledge_case = full_knowledge_case;
            
            case 3
                disp('Case3: Execute and update sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                execute_update_case = case3(datasets, init_values, problem_setup);
                all_cases.execute_update_case = execute_update_case;
            
            case 4
                disp('Case4: Execute and selectively predict sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                selective_prediction_case = case4(datasets, init_values, problem_setup, true);
                all_cases.selective_prediction_case = selective_prediction_case;

            case 5
                disp('Case5: Execute and selectively update sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                collision_t_update_case = case5(datasets, init_values, problem_setup);
                all_cases.collision_t_update_case = collision_t_update_case;
  
            case 6
                % This should be the same result as full knowledge but MUCH slower
                disp('Case6: Execute and predict sdf');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                prediction_case = case6(datasets, init_values, problem_setup, true);
                all_cases.prediction_case = prediction_case;
           
            case 7
                disp('Case7: Execute and update sdf using reinitialisation');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                execute_update_case_reinit = case7(datasets, init_values, problem_setup);
                all_cases.execute_update_case_reinit = execute_update_case_reinit;
            
            case 8
                disp('Case8: Execute and update sdf using pruning');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                pruning_reinit_case = case8(datasets, init_values, problem_setup);
                all_cases.pruning_reinit_case = pruning_reinit_case;
            
            case 9
                disp('Case9: Manual fast predictions sdfs');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                fast_prediction_manual_case = case9(datasets, init_values, problem_setup);
                all_cases.fast_prediction_manual_case = fast_prediction_manual_case;
            
            case 10
                disp('Case10: Manual full knowledge case');
                init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);
                full_knowledge_manual_case = case10(datasets, init_values, problem_setup);
                all_cases.full_knowledge_manual_case = full_knowledge_manual_case;
        end
    end

end

