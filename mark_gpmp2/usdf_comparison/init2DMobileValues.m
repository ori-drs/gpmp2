function init_values = init2DMobileValues(problem_setup)

    % initial values
    init_values = gtsam.Values;
    
    avg_vel = [problem_setup.end_pose.x()-problem_setup.start_pose.x(); ...
        problem_setup.end_pose.y()-problem_setup.start_pose.y(); ...
        problem_setup.end_pose.theta()-problem_setup.start_pose.theta()] / problem_setup.delta_t;


    for i = 0 : problem_setup.total_time_step
        key_pos = gtsam.symbol('x', i);
        key_vel = gtsam.symbol('v', i);

        % initialize as straight line in conf space
        pose = gtsam.Pose2(problem_setup.start_pose.x() * (problem_setup.total_time_step-i)/problem_setup.total_time_step + ...
            problem_setup.end_pose.x() * i/problem_setup.total_time_step, ...
            problem_setup.start_pose.y() * (problem_setup.total_time_step-i)/problem_setup.total_time_step + ...
            problem_setup.end_pose.y() * i/problem_setup.total_time_step, ...
            problem_setup.start_pose.theta() * (problem_setup.total_time_step-i)/problem_setup.total_time_step + ...
            problem_setup.end_pose.theta() * i/problem_setup.total_time_step);
        vel = avg_vel;

        init_values.insert(key_pos, pose);
        init_values.insert(key_vel, vel);
    end
end

