function problem_setup = paperGetProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier, limit_x, limit_v)
                                     
%PAPERGETPROBLEMSETUP Summary of this function goes here
%   Detailed explanation goes here

    arm = gpmp2.generateArm('WAMArm');
    arm_model = arm.fk_model();

    total_time_step = round(total_time_sec/delta_t);
    total_check_step = interp_multiplier*total_time_step;
    check_inter = total_check_step / total_time_step - 1;
    
    use_trustregion_opt = false;

    start_vel = zeros(7,1);
    end_vel = zeros(7,1);
    
    % noise model
    pose_fix_sigma = 0.0001;
    vel_fix_sigma = 0.0001;

    pose_fix_model = gtsam.noiseModel.Isotropic.Sigma(7, pose_fix_sigma);
    vel_fix_model = gtsam.noiseModel.Isotropic.Sigma(7, vel_fix_sigma);
    
    % use GP interpolation
    use_GP_inter = true;

    % GP
    Qc = 1 * eye(7);
    Qc_model = gtsam.noiseModel.Gaussian.Covariance(Qc); 
    
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
    problem_setup.limit_x = limit_x;    
    problem_setup.limit_v = limit_v;
end

