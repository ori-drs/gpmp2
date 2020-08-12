
env_name = "MovingReplannerOneBlock"
env = loadSDFAnalysisEnvironment(env_name, 96, 0.04);
env.queryEnv(0)



[X, Y, Z] = getEnvironmentMesh(env.getDataset);
title("Graph");
xlabel('x'); ylabel('y'); zlabel('z'); view(3);
ax_lims = [-1 1.5 -1.2 1.5 -1 2];
axis(ax_lims);
h1 = plot3DEnvironment(env.getDataset, X, Y, Z);
% 

