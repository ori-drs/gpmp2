function problem_setup = mobileProblemSetup2D(start_pose, end_pose, total_time_sec, delta_t, ...
                                         cost_sigma, epsilon_dist, interp_multiplier)
                                     
%PAPERGETPROBLEMSETUP Summary of this function goes here
%   Detailed explanation goes here
    import gtsam.*
    import gpmp2.*
        
    spheres_data = [...
        0  0.0  0.0  0.0  0.2];
    nr_body = size(spheres_data, 1);
    sphere_vec = BodySphereVector;
    for i=1:nr_body
        sphere_vec.push_back(BodySphere(spheres_data(i,1), spheres_data(i,5), ...
            Point3(spheres_data(i,2:4)')));
    end
    robot = Pose2MobileBaseModel(Pose2MobileBase, sphere_vec);
    
    % GP
    Qc = 1 * eye(robot.dof());
    Qc_model = noiseModel.Gaussian.Covariance(Qc);

    % prior to start/goal
    pose_fix_sigma = 0.0001;
    vel_fix_sigma = 0.0001;
    pose_fix_model = noiseModel.Isotropic.Sigma(robot.dof(), pose_fix_sigma);
    vel_fix_model = noiseModel.Isotropic.Sigma(robot.dof(), vel_fix_sigma);

    total_time_step = round(total_time_sec/delta_t);
    total_check_step = interp_multiplier*total_time_step;
    check_inter = total_check_step / total_time_step - 1;
    
    use_trustregion_opt = false;
    use_LM = false;
    
    start_vel = [0, 0, 0]';
    end_vel = [0, 0, 0]';
    
    problem_setup.start_pose = start_pose;
    problem_setup.end_pose = end_pose;
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
    problem_setup.robot = robot;
    problem_setup.Qc_model = Qc_model;
    problem_setup.use_trustregion_opt = use_trustregion_opt;
    problem_setup.use_LM = use_LM;
end

