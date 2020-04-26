function [env] = loadPredefinedMovingEnvironment(env_name)
%LOADPREDEFINEDMOVINGENVIRONMENT Summary of this function goes here
%   Detailed explanation goes here

if strcmp(env_name, 'MovingReplanner')

    v_or_t_end = true;
    starting_pos1 = [0.55, 0.25, -0.05];
    v_or_t_end_value1 = [-0.6, 0, 0];
    starting_pos2 = [0.55, -0.95, -0.05];
    v_or_t_end_value2 = [0,0.6, 0];
    obs_size = [0.15, 0.15, 1.9];

    % Create the environment
    env = movingEnvironment3D();
    env.add_static_scene();
    env.add_object(0,...
                    v_or_t_end, ...
                    v_or_t_end_value1, ...
                    starting_pos1, ...
                    obs_size);

    env.add_object(0,...
                    v_or_t_end, ...
                    v_or_t_end_value2, ...
                    starting_pos2, ...
                    obs_size);
                
elseif strcmp(env_name, 'MovingBlock')
    % Setup
    v_or_t_end = true;
    v_or_t_end_value = [0,-0.16, 0];
    starting_pos = [0.40, 0.4, 0.4];
    obs_size = [0.2, 0.2, 0.2];

    % Create the environment
    env = movingEnvironment3D();
    env.add_object(0,...
                    v_or_t_end, ...
                    v_or_t_end_value, ...
                    starting_pos, ...
                    obs_size);   
    
end

end

