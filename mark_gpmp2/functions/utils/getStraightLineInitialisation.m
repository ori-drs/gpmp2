function init_values = getStraightLineInitialisation(start_conf, end_conf, total_time_step, delta_t)
%GETSTRAIGHTLINEINITIALISATION Summary of this function goes here
%   Detailed explanation goes here
    
    init_values = gtsam.Values;

    % For 
    for i = 0 : total_time_step
        key_pos = gtsam.symbol('x', i);
        key_vel = gtsam.symbol('v', i);

        % initialize as straight line in conf space
        pose = start_conf * (total_time_step-i)/total_time_step + end_conf * i/total_time_step;
        vel = (end_conf / total_time_step) / delta_t;

        init_values.insert(key_pos, pose);
        init_values.insert(key_vel, vel);
    end

end

