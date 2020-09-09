function [env] = loadSDFAnalysisEnvironment(env_name, env_size, res)
%LOADPREDEFINEDMOVINGENVIRONMENT Summary of this function goes here
%   Detailed explanation goes here

if strcmp(env_name, 'MovingReplannerOneBlock')

    starting_pos1 = [0.55, -0.95, -0.05];
    block_vel1 = [-0.17,0.6, 0];
    obs_size = [0.15, 0.15, 1.9];

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_static_scene();
    env.add_object(0,...
                    2, ...
                    block_vel1, ...
                    starting_pos1, ...
                    obs_size);

elseif strcmp(env_name, 'MovingReplanner')

    starting_pos1 = [0.3, 0.25, -0.05];
    block_vel1 = [-0.6, 0, 0];
    starting_pos2 = [0.55, -0.95, -0.05];
    block_vel2 = [-0.17,0.6, 0];
    obs_size = [0.15, 0.15, 1.9];

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_static_scene();
    env.add_object(0,...
                    0, ...
                    block_vel1, ...
                    starting_pos1, ...
                    obs_size);

    env.add_object(0,...
                    2, ...
                    block_vel2, ...
                    starting_pos2, ...
                    obs_size);
                
elseif strcmp(env_name, 'MovingReplannerSmallBlocks')

    starting_pos1 = [0.3, 0.25, 1.45];
    block_vel1 = [-0.6, 0, 0];
    starting_pos2 = [0.55, -0.95, 1.45];
    block_vel2 = [-0.17,0.6, 0];
    obs_size = [0.15, 0.15, 0.15];

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_static_scene();
    env.add_object(0,...
                    0, ...
                    block_vel1, ...
                    starting_pos1, ...
                    obs_size);

    env.add_object(0,...
                    2, ...
                    block_vel2, ...
                    starting_pos2, ...
                    obs_size);

                
elseif strcmp(env_name, 'MovingReplannerNoStatic')

    starting_pos1 = [0.3, 0.25, -0.05];
    block_vel1 = [-0.6, 0, 0];
    starting_pos2 = [0.55, -0.95, -0.05];
    block_vel2 = [-0.17,0.6, 0];
    obs_size = [0.15, 0.15, 1.9];

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_object(0,...
                    0, ...
                    block_vel1, ...
                    starting_pos1, ...
                    obs_size);

    env.add_object(0,...
                    2, ...
                    block_vel2, ...
                    starting_pos2, ...
                    obs_size);

                
elseif strcmp(env_name, 'MovingBlock')
    % Setup
    t_end_moving = 0;
    v_or_t_end_value = [-0.17,0.6, 0];
    starting_pos = [0.40, 0.4, 0.4];
    obs_size = [0.2, 0.2, 0.2];

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_object(0,...
                    t_end_moving, ...
                    v_or_t_end_value, ...
                    starting_pos, ...
                    obs_size);   
    
end

if strcmp(env_name, 'Lab')

    % Create the environment
    env = movingEnvironment3D(env_size, res);
    env.add_static_scene();

end
