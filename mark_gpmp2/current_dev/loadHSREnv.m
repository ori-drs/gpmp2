function [env] = loadHSREnv(env_size, res, speed)

hsr_size = [0.48, 0.55, 1.2];
hsr_pos = [0.4, 2.0, 0.6];
hsr_vel = [0, -1, 0] * speed;

env = movingEnvironment3D(env_size, res);

env.add_hsr_static_scene();

env.add_object(0,...
                3, ...
                hsr_vel, ...
                hsr_pos, ...
                hsr_size);
                    

                    
end

