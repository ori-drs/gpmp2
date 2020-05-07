% 
% @author Mark Finean 
% @date May 03, 2020

close all;
clear all;
clc;

import gtsam.*
import gpmp2.*


%% Setup

% env_name = 'MovingReplanner';
env_name = 'Lab';
env = loadPredefinedMovingEnvironment(env_name);
dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);

% map3D = binaryOccupancyMap(dataset.map);
resolution = 100; %cells per metre
map3D = occupancyMap3D(resolution);

% surf1=isosurface(X, Y, Z, data, isovalue);

X = reshape(X,1,[])';
Y = reshape(Y,1,[])';
Z = reshape(Z,1,[])';

% Offsets to account for the mounted base
% X = X + 0.14;
% Y = Y + 0.22;
% Z = Z + 0.346;

occupancy_vector = reshape(dataset.map,1,[])';   

%Find occupied cells
occupancy_inds = find(occupancy_vector==1);
occupancy_vector = occupancy_vector(occupancy_inds);
X = X(occupancy_inds);
Y = Y(occupancy_inds);
Z = Z(occupancy_inds);

pose = [ 0 0 0 1 0 0 0];
points = [X, Y, Z];

maxRange = 3;

insertPointCloud(map3D,pose,points,maxRange);
show(map3D)

% Export file
dir_path = "/home/mark/installs/gpmp2/mark_gpmp2/data/";
% file_name = dir_path + "moving_replanner_octomap_time_0.bt";
file_name = dir_path + "lab_octomap_time_0.bt";
exportOccupancyMap3D(map3D,file_name)
% 
% msg = rosmessage('nav_msgs/OccupancyGrid');
% 
% msg.Data = occupancy_vector;
% msg.Info.Height = 3.0;
% msg.Info.Width = 3.0;
% msg.Info.Origin.Position.X = -1;
% msg.Info.Origin.Position.Y = -1;
% msg.Info.Origin.Position.Z = -1;
% msg.Info.Origin.Orientation.W = 1;
% msg.Info.Origin.Orientation.Z = 0;
% msg.Info.Origin.Orientation.Y = 0;
% msg.Info.Origin.Orientation.X = 0;
% msg.Info.Resolution = 0.01;