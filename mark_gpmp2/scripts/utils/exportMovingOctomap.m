% 
% @author Mark Finean 
% @date May 03, 2020

close all;
clear all;
clc;

import gtsam.*
import gpmp2.*


%% Setup

env_name = 'MovingReplanner';
% env_name = 'MovingBlock';
env = loadPredefinedMovingEnvironment(env_name, 300 , 0.01);
dataset1 = env.queryEnv(0);
dataset2 = env.queryEnv(0.1);
%%
tic;
field  = gpmp2.signedDistanceField3D(dataset2.map, 0.01); 
toc;                                                        

%%
data1_path = "/home/mark/installs/gpmp2/mark_gpmp2/data/data1_moving_replanner.txt";
data2_path = "/home/mark/installs/gpmp2/mark_gpmp2/data/data2_moving_replanner.txt";

file1_ID = fopen(data1_path,'w');
file2_ID = fopen(data2_path,'w');

points1 = getCoords(dataset1);
points2 = getCoords(dataset2);

% Note: need arrays in y column order here
nbytes_1 = fprintf(file1_ID,'%3f %3f %3f\n',points1');
nbytes_2 = fprintf(file2_ID,'%3f %3f %3f\n',points2');

function points = getCoords(dataset)
    [X, Y, Z] = getEnvironmentMesh(dataset);
    X = reshape(X,1,[])';
    Y = reshape(Y,1,[])';
    Z = reshape(Z,1,[])';

    occupancy_vector = reshape(dataset.map,1,[])';   

    %Find occupied cells
    occupancy_inds = find(occupancy_vector==1);
    X = X(occupancy_inds);
    Y = Y(occupancy_inds);
    Z = Z(occupancy_inds);

    points = [X, Y, Z];
end

% 
% insertPointCloud(map3D,pose,points,maxRange);
% show(map3D)
% 
% % Export file
% dir_path = "/home/mark/installs/gpmp2/mark_gpmp2/data/";
% % file_name = dir_path + "moving_replanner_octomap_time_0.bt";
% file_name = dir_path + "lab_octomap_with_z_offset_time_0.bt";
% exportOccupancyMap3D(map3D,file_name)
