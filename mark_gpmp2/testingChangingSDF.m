clear all;
close all;
clc;

import gtsam.*
import gpmp2.*


dataset1 = generate2Ddataset('OneObstacleDataset');
dataset2 = generate2Ddataset('TwoObstaclesDataset');

% all(all(dataset1.map == dataset2.map))

rows = dataset1.rows;
cols = dataset1.cols;
cell_size = dataset1.cell_size;
origin_point2 = Point2(dataset1.origin_x, dataset1.origin_y);

% signed distance field
field1 = signedDistanceField2D(dataset1.map, cell_size);
sdf1 = PlanarSDF(origin_point2, cell_size, field1);

field2 = signedDistanceField2D(dataset2.map, cell_size);
sdf2 = PlanarSDF(origin_point2, cell_size, field2);

% plot sdf
figure(1)
subplot(1,2,1);
plotSignedDistanceField2D(field1, dataset1.origin_x, dataset1.origin_y, dataset1.cell_size);
title('Signed Distance Field')

% plot sdf
subplot(1,2,2);
plotSignedDistanceField2D(field2, dataset1.origin_x, dataset1.origin_y, dataset1.cell_size);
title('Signed Distance Field')


% sdf1.getSignedDistance(gtsam.Point2(0.3, 0.8))
% sdf2.getSignedDistance(gtsam.Point2(0.3, 0.8))
% 
% sdf2.changeData(field1);
% 
% sdf1.getSignedDistance(gtsam.Point2(0.3, 0.8))
% sdf2.getSignedDistance(gtsam.Point2(0.3, 0.8))
% 
% figure(2)
% subplot(1,2,1);
% plotSignedDistanceField2D(field1, dataset1.origin_x, dataset1.origin_y, dataset1.cell_size);
% title('Signed Distance Field')
% 
% % plot sdf
% subplot(1,2,2);
% plotSignedDistanceField2D(field2, dataset1.origin_x, dataset1.origin_y, dataset1.cell_size);
% title('Signed Distance Field')


key_pos1 = gtsam.symbol('x', 0);
arm = gpmp2.generateArm('SimpleTwoLinksArm');
cost_sigma = 0.1;
epsilon_dist = 0.1;
fact = gpmp2.ObstaclePlanarSDFFactorArm(key_pos1, ...
                                        arm, ...
                                        sdf1, ...
                                        cost_sigma, ...
                                        epsilon_dist);




graph = gtsam.NonlinearFactorGraph;
init_values = gtsam.Values;
key_pos = gtsam.symbol('x', 0);
graph.add(fact);

field3 = signedDistanceField2D(dataset2.map, cell_size);

% graph.add(gpmp2.ObstaclePlanarSDFFactorArm(...
%           key_pos, arm, dataset.sdf, cost_sigma, epsilon_dist));
% 
%       
      
graph.at(0).changeSDFData(field3);
% 
disp("it worked");
