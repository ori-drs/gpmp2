
clear all;
close all;
clc;

% rosinit
node = ros.Node('/test');

cell_size = 0.01;
dataset_size = [150,150,150];

map = zeros(dataset_size);
% sub = ros.Subscriber(node,'/hsrb/head_rgbd_sensor/depth_registered/points','sensor_msgs/PointCloud2');
% sub = ros.Subscriber(node,'/camera/depth/points','sensor_msgs/PointCloud2');
sub = ros.Subscriber(node,'/octomap_point_cloud_centers','sensor_msgs/PointCloud2');
sub = ros.Subscriber(node,'/octomap_point_cloud_centers','sensor_msgs/PointCloud2');
octo_sub = ros.Subscriber(node,'/octomap_full','octomap_msgs/Octomap');

octomap_msg = octo_sub.LatestMessage;
octomap = readOccupancyMap3D(octomap_msg)'
pause(2);

tic;
ptcloud = sub.LatestMessage;
xyz = readXYZ(ptcloud);
toc;

tic;
ptcloud = sub.LatestMessage;
xyz = readXYZ(ptcloud);
rgb = uint8(255*readRGB(ptcloud));
pcobj = pointCloud(xyz,'Color', rgb);
pcobj = pcdownsample(pcobj,'gridAverage',0.02);
toc;

xyz = pcobj.Location;

lower = min(xyz);
upper = max(xyz);
  
xlimits = [lower(1) upper(1)];
ylimits = [lower(2) upper(2)];
zlimits = [lower(3) upper(3)];

% showPointCloud(pcobj)

% player = pcplayer(xlimits,ylimits,zlimits);
% view(player,pcobj);    
% % scatter3(ptcloud)

origin = [-0.5,-0.5,1.3];
max_coord = origin + dataset_size * cell_size;

tic;
inds_in_bounds = all(xyz >= origin & xyz <= max_coord,2);

occupied_xyz = xyz(inds_in_bounds, :);
occupied_rgb = rgb(inds_in_bounds, :);

cropped_pcobj = pointCloud(occupied_xyz,'Color',uint8(255*occupied_rgb));
figure(1); hold on;
pcshow(cropped_pcobj)
title('Lidar point cloud with intensity')
xlabel('X')
ylabel('Y')
zlabel('Z')
colorbar
    
occupied_voxel_inds = xyz_to_voxel_inds(occupied_xyz, origin, cell_size, dataset_size);
map(occupied_voxel_inds) = 1;
toc;


field  = gpmp2.signedDistanceField3D(map, cell_size); 
% Plot the real sdf
figure(2); hold on;
gpmp2.plotSignedDistanceField3D(permute(field, [2 1 3]), origin, cell_size);
colormap(jet)

function [inds] = xyz_to_voxel_inds(xyz_points, origin, cell_size, m_size)
    % Convert xyz to a voxel coordinate
    cell_coords = round((xyz_points - origin) / cell_size) + [1,1,1];

    % Ensure in bounds    
    cell_coords = cell_coords(all(cell_coords>0 & cell_coords<=m_size, 2), :);
    
    % Convert coords to inds
    inds = sub2ind(m_size, cell_coords(:,1)', cell_coords(:,2)', cell_coords(:,3)');

    % Make unique    
    inds = unique(inds);
end

