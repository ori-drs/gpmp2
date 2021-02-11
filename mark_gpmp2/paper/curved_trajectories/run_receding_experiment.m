clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

plot_graphs = false;
total_time_sec = 3;
delta_t = 0.1;
interp_multiplier = 1;
cost_sigma = 0.05;
epsilon_dist = 0.3;    
limit_v = false;
limit_x = false;

cell_size = 0.04;
env_size = 200;
origin = [-1,-1,-1];
base_pos = [-0.05, 0, 0.4];

total_time_step = round(total_time_sec/delta_t);

% Setup problem
start_conf = setPandaConf('table_forward');
end_conf = setPandaConf('table_forward');

r = 2;
speed = 2;
trajCalculator = recedingCurvedTrajectory(speed, r);

        
%% This is repeated for each experiment

problem_setup = pandaProblemSetup(start_conf, end_conf, total_time_sec, delta_t, ...
                                 cost_sigma, epsilon_dist, interp_multiplier, ...
                                 limit_x, limit_v, base_pos);

init_values = gpmp2.initArmTrajStraightLine(start_conf, end_conf, total_time_step);

env = curvedEnvironment(env_size, cell_size, origin);
start_sdf = env.getSDF();
panda_planner = recedingPandaPlanner(start_sdf, problem_setup);
tracker = liveTracker([env_size,env_size,env_size], env.dataset.static_map, ...
                        epsilon_dist, cell_size, env.dataset.origin_point3);

sdfs  = cell(total_time_step, 1);
for i = 1:total_time_step+1
    sdfs{i} = gpmp2.SignedDistanceField(env.dataset.origin_point3, ...
                                                cell_size, ...
                                                env_size, ...
                                                env_size, ...
                                                env_size);
end

pause(1);

env.updateMap(trajCalculator.getPosition(0));

% First optimisation
result = panda_planner.optimize(init_values);
%%%

[X, Y, Z] = getEnvironmentMesh(env.dataset);
figure(3); hold on; cla;
lab_axis_lims = [-1 2 -1 2 -1 2];
%     set(gcf,'Position',[1350 500 1200 1400]);
axis(lab_axis_lims); 
grid on; 
view(3);
xlabel('x'); ylabel('y'); zlabel('z');
%%%

t_step = 1;

num_steps_in_collision = 0;

for time_steps = 0:100 
    
    block_pos = trajCalculator.getPosition(t_step * delta_t);
    
    env.updateMap(block_pos);
    
    tracker.update(t_step * delta_t, env.dataset.map);

    curr_conf = result.atVector(gtsam.symbol('x', 1));
    
    panda_planner.graph.replace(0, gtsam.PriorFactorVector(gtsam.symbol('x', 0), result.atVector(gtsam.symbol('x', 1)), problem_setup.pose_fix_model));
    panda_planner.graph.replace(1, gtsam.PriorFactorVector(gtsam.symbol('v', 0), result.atVector(gtsam.symbol('v', 1)), problem_setup.pose_fix_model));
        

    for t = 0:total_time_step
        forward_t = t*delta_t;

        field = tracker.predict_composite_sdf(forward_t);

        for z = 1:env_size
            sdfs{t+1}.initFieldData(z-1, field(:,:,z)');
        end

        panda_planner.update_sdf(t, sdfs{t+1});
    end
    
    init_values = gpmp2.initArmTrajStraightLine(curr_conf, end_conf, total_time_step);

    result = panda_planner.optimize(init_values);

    
    %%%
    
    cla;
    traj = result;

    h1 = plot3DEnvironment(env.dataset.map, X, Y, Z);
    
    %%% Plot circle
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + 0.4 + r;
    yunit = r * sin(th);
    h = plot(xunit, yunit);
    %%% 


    key_pos = gtsam.symbol('x',0);
    conf = traj.atVector(key_pos);
    
    obs_factor = gpmp2.ObstacleSDFFactorArm(...
        key_pos, problem_setup.arm, env.getSDF(), problem_setup.cost_sigma, ...
        0);
    
    num_steps_in_collision =  num_steps_in_collision + sum(any(obs_factor.evaluateError(result.atVector(key_pos))));

    h1 = plot3DEnvironment(env.dataset.map, X, Y, Z);

    static_handle = gpmp2.plotRobotModel(problem_setup.arm, conf);

    pause(0.05);
    t_step = t_step + 1;
end 