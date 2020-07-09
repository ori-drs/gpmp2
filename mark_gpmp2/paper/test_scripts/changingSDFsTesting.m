cost_sigma = 0.03;
epsilon_dist = 0.25;    

arm = generateArm('Panda');
pose_key = gtsam.symbol('x', 0);


env = loadPredefinedMovingEnvironment('MovingReplanner', 150, 0.02, [-1,-1,-1]);
dataset = env.queryEnv(0);

env2 = loadPredefinedMovingEnvironment('MovingReplanner', 150, 0.02, [-1,-1,-1]);
dataset2 = env.queryEnv(1);


factor1 = gpmp2.ObstacleSDFFactorArm(pose_key, ...
            arm, ...
            dataset1.sdf, ...
            cost_sigma, ...
            epsilon_dist);
        
factor2 = gpmp2.ObstacleSDFFactorArm(pose_key, ...
            arm, ...
            dataset2.sdf, ...
            cost_sigma, ...
            epsilon_dist);
        
        
factor_comp = gpmp2.ObstacleSDFFactorArm(pose_key, ...
            arm, ...
            dataset1.sdf, ...
            cost_sigma, ...
            epsilon_dist); 
        

tic;
new_fact = factor.getSDFModFactor(dataset2.sdf);            
toc;


% tic;
% factor.replaceSDFData(dataset2.sdf);
% toc;

tic;
new_fact = factor.getSDFModFactor(dataset2.sdf);            
toc;