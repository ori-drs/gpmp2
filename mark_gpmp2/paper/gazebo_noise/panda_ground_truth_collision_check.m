import gpmp2.*
import gtsam.*

% HSR trajectory parameters
start_pos = [0.4, 2.0, 0];
velocity = [0, -1.2, 0];

% Environment parameters 
cell_size = 0.04;
env_size = 64;
origin = [-1,-1,-1];
scene = "tables"; 
obstacle = "hsrb";
base_pos = [0, 0, 0.4];

% Create arm
arm = generateArm('Panda', Pose3(Rot3(eye(3)), Point3(base_pos')));

%  Our experiment parameter weights
cost_sigma = 0.05;
epsilon_dist = 0.3;    

% Create the environment
env = rebuttalGroundTruthEnvironment(env_size, cell_size, origin, obstacle, scene);
env.add_table_static_scene();

% To store the sdfs
sdfs = cell(1,31);
obs_factors = cell(1,31);

% Calculate the ground truth SDFs
i=1;
for t = 0:0.1:3.0
    key_pos = gtsam.symbol('x', i);
    pos = start_pos + t*velocity;
    env.updateMap(pos);
    sdfs{i} = env.getSDF();
    obs_factors{i} = gpmp2.ObstacleSDFFactorArm(key_pos, arm, sdfs{i}, ...
                                                cost_sigma, epsilon_dist);
    i = i + 1;
end

obstacle_graph = pandaObstacleGraph(sdfs, problem_setup);
smoothness_graph = pandaSmoothnessGraph(problem_setup);