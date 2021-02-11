% HSR Test example
% @author Mark Finean

close all
clear

import gtsam.*
import gpmp2.*


%% dataset
dataset = generate3Ddataset('SmallDemo');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
% cell_size = dataset.cell_size;
cell_size = dataset.cell_size;

% arm: WAM arm
marm = generateMobileArm('HSR');
marm_fk_model = marm.fk_model();

start_pose = Pose2(0, 0, 0);

start_conf = [0.5,0,0,-1.57,0]';
end_conf =  [0.1,0.7,0,-1.57,0]';

pstart = Pose2Vector(start_pose, start_conf);

% plot problem setting
figure(1), hold on
title('Problem Settings')
% plotMap3D(dataset.corner_idx, origin, cell_size);
plotMobileRobotModel(marm, pstart, [0.4 0.2], 'b', 1)
% plotMobileRobotModel(marm, pend)
% plot config
xlabel('X');ylabel('Y');zlabel('Z');
axis([-2 2 -2 2 -0.4 2])
grid on, view(3)
hold on

%%

cla;
start_pose = gtsam.Pose2(0, 0, 0);
start_conf = [0.3,0,0,0,0,0]';

pstart = gpmp2.Pose2Vector(start_pose, start_conf);

gpmp2.plotMobileRobotModel(marm, pstart, [0.4 0.2], 'b', 1)

