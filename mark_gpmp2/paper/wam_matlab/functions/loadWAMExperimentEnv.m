function [env] = loadWAMExperimentEnv(env_num, env_size, res, speed)
%LOADPAPERENVS Summary of this function goes here
%   Detailed explanation goes here

pillar_size = [0.15, 0.15, 1.9];
pillar_2_pos = [0.55, -0.95, -0.05];
pillar_2_vel = [-0.2726, 0.9621, 0] * speed; 

env = movingEnvironment3D(env_size, res);

switch env_num

    case 1
        % Just the second pillar
        env.add_static_scene();

        env.add_object(0,...
                        2, ...
                        pillar_2_vel, ...
                        pillar_2_pos, ...
                        pillar_size);
                    
    case 4
        % Just the second pillar
        env.add_static_scene();

        env.add_object(0,...
                        2, ...
                        pillar_2_vel, ...
                        pillar_2_pos, ...
                        pillar_size);
end
                    
end

