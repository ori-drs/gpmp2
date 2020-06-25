function [env] = loadPaperEnvs(env_num, env_size, res, speed)
%LOADPAPERENVS Summary of this function goes here
%   Detailed explanation goes here

pillar_size = [0.15, 0.15, 1.9];
block_size = [0.2, 0.2, 0.2];

pillar_1_pos = [0.3, 0.25, -0.05];
pillar_2_pos = [0.55, -0.95, -0.05];
block_pos = [0.40, 0.4, 0.4];

pillar_1_vel = [-1, 0, 0] * speed;
pillar_2_vel = [-0.2726, 0.9621, 0] * speed; 
block_vel = [-0.2726, 0.9621, 0] * speed;

env = movingEnvironment3D(env_size, res);

switch env_num

    case 1
        % Just the second block
        env.add_static_scene();

        env.add_object(0,...
                        2, ...
                        pillar_2_vel, ...
                        pillar_2_pos, ...
                        pillar_size);
                    
    case 4
        % Just the second block
        env.add_static_scene();

        env.add_object(0,...
                        2, ...
                        pillar_2_vel, ...
                        pillar_2_pos, ...
                        pillar_size);
end



% 
% % MovingReplanner reverse
% env.add_static_scene();
% env.add_object(0,...
%                 0, ...
%                 -pillar_1_vel, ...
%                 pillar_1_pos + 3 * pillar_1_vel * 0.6/speed, ...
%                 pillar_size);
% 
% env.add_object(1,...
%                 0, ...
%                 -pillar_2_vel, ...
%                 pillar_2_pos + 2 * pillar_2_vel * 0.6/speed, ...
%                 pillar_size);  
                    
                    
% case 1
%     % MovingReplanner
%     env.add_static_scene();
%     env.add_object(0,...
%                     0, ...
%                     pillar_1_vel, ...
%                     pillar_1_pos, ...
%                     pillar_size);
% 
%     env.add_object(0,...
%                     2, ...
%                     pillar_2_vel, ...
%                     pillar_2_pos, ...
%                     pillar_size);
% case 2
% 
% case 3
%     % MovingBlock
%     env.add_object(0,...
%                     0, ...
%                     block_vel, ...
%                     block_pos, ...
%                         block_size);   
                    
end

