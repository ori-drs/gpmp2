
% @author Mark Finean 
% @date June 01, 2020

close all;
clear all;
clc;
    
import gpmp2.*
import gtsam.*

% Run the lab experiments
env_size = 150;
res = 0.02;
speed = 0;

start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]'; % default
start_conf = [-0.4,-1.70,1.64,1.45,1.1,-0.206,2.2]';

end_conf = [0.0,0.94,0,1.6,0,-0.919,1.55]'; % middle shelf
end_conf = [0.0,0.6,0,0.9,0,-0.119,1.55]'; % top shelf
end_conf = [-0.4,0.94,0,1.6,-0.3,-0.919,1.55]'; % right middle shelf

start_conf = [-0.4,-1.70,1.64,1.45,1.1,-0.206,2.2]';
end_conf = [-0.4,0.94,0,1.6,-0.3,-0.919,1.55]'; % middle shelf

arm = gpmp2.generateArm('WAMArm');
% arm_model = arm.fk_model();

env = loadPaperEnvs(1, env_size, res, speed);

dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);
lab_axis_lims = [-1 1.5 -1.2 1.5 -1 2];

%% Solve for each scenario
import gpmp2.*
import gtsam.*

rotate3d on
figure(1); hold on; 
set(gcf,'Position',[1350 500 1200 1400]);
title('3D Environment')
grid on;
view(3);
axis(lab_axis_lims);
xlabel('x'); ylabel('y'); zlabel('z');
plot3DEnvironment(dataset, X, Y, Z)
% gpmp2.plotArm(arm.fk_model(), ...
%         start_conf, 'b', 2)
% gpmp2.plotArm(arm.fk_model(), ...
%         end_conf, 'r', 2)
gpmp2.plotRobotModel(arm, ...
        start_conf)
gpmp2.plotRobotModel(arm, ...
        end_conf)
