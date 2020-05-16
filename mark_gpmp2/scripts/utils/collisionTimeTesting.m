clear all;
close all;
clc;

import gtsam.*
import gpmp2.*

v_or_t_end = true;
starting_pos1 = [0.55, 0.25, -0.05];
v_or_t_end_value1 = [-0.6, 0, 0];
starting_pos2 = [0.55, -0.95, -0.05];
v_or_t_end_value2 = [0,0.6, 0];
obs_size = [0.15, 0.15, 1.9];

% Create the environment
env = movingEnvironment3D();
% env.add_static_scene();
env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value1, ...
                starting_pos1, ...
                obs_size);

env.add_object(0,...
                v_or_t_end, ...
                v_or_t_end_value2, ...
                starting_pos2, ...
                obs_size);

dataset = env.queryEnv(0);
[X, Y, Z] = getEnvironmentMesh(dataset);

%% get datasets

disp('Getting all the datasets');
total_time_step = 20;
delta_t = 0.1;
datasets = [];

for i = 0:total_time_step
    t = i *  delta_t;
    dataset = env.queryEnv(t);
    datasets = [datasets, dataset];
end   
%%

total_time = total_time_step*delta_t;
dataset_size = [300,300,300];


new_map = datasets(1).map;
conn_comps = bwconncomp(new_map);
regions = regionprops(conn_comps,'Centroid');
num_obs = conn_comps.NumObjects;

px_ind_list = conn_comps.PixelIdxList;
px_list = px_ind_list{1};

collision_t_mapping = repelem([0:delta_t:total_time]', size(px_list,1));


tic;
[row,col,z] = ind2sub(dataset_size,px_list);
obj_coords = horzcat(row,col,z);

test_px_vel = [20,0,0];

vel_rep_matrix = repmat(test_px_vel, 1, total_time_step+1);
all_times = repelem([0:delta_t:total_time], 3);

delta_px = vel_rep_matrix .* all_times; % This gives the change in pix for all combos

all_coords = repmat(obj_coords, 1, total_time_step+1);
all_coords = all_coords + delta_px; % All pixel coords in collision over the trajectory

% Need to remove pixels out of bounds
x_coords = all_coords(:,1:3:end);
y_coords = all_coords(:,1:3:end);
z_coords = all_coords(:,1:3:end);

in_bound_mask = (1 <= x_coords) & (x_coords <= dataset_size(1)) & ...
                (1 <= y_coords) & (y_coords <= dataset_size(2)) & ...
                (1 <= z_coords) & (z_coords <= dataset_size(3)) ;
            
collision_t_mapping = collision_t_mapping(in_bound_mask);

px_inds = sub2ind(dataset_size,...
                reshape(x_coords(in_bound_mask), [], 1)', ...
                reshape(y_coords(in_bound_mask), [], 1)', ...
                reshape(z_coords(in_bound_mask), [], 1)')';
toc;


% 
% t = 0.7;                
% i = 3;
% 
% D_change = (datasets(i+1).field - datasets(i).field)/delta_t;
% inds_ignore = D_change>0.95*min(min(min(D_change))); % negative/lowest change means objects heading towards
% time_to_collision = -datasets(i+1).field./D_change;
% time_to_collision(time_to_collision<0) = -1; % Never going to collide;
% time_to_collision(time_to_collision> 5 ) = -1; % collision a long time away
% % time_to_collision(time_to_collision>100)=100;
% % time_to_collision(inds_ignore)=100;
% 
% 
% 
% diff_map_now = datasets(i+1).map - flip(datasets(i+1).static_map);
% diff_map_before = datasets(i).map - flip(datasets(i).static_map);
% 
% field_now  = gpmp2.signedDistanceField3D(permute(diff_map_now, [2 1 3]), 0.01);   
% field_before  = gpmp2.signedDistanceField3D(permute(diff_map_before, [2 1 3]), 0.01);   
%                                                                                                                 
% D_change = (field_now - field_before)/delta_t;
% time_to_collision = -datasets(i+1).field./D_change;
% time_to_collision(time_to_collision<0) = 0; % Never going to collide;
% time_to_collision(time_to_collision> 10 ) = 0; % collision a long time away
% 
% D_change(D_change<=0) = 10;
% 
% 
% 
% 
% figure(1); imagesc(datasets(i+1).field(:,:,150)); colorbar; title('field now');
% figure(2); imagesc(datasets(i).field(:,:,150)); colorbar; title('previous now');
% figure(3);hold on; view(3); grid on; axis([-0.5 1 -0.5 0.5 -0.2 1]); title('scene');
% h1 = plot3DEnvironment(datasets(i+1).map, X, Y, Z);
% figure(4); imagesc(D_change(:,:,150)); colorbar; title('D_change');
% figure(5); imagesc(time_to_collision(:,:,150)); colorbar; title('time to collision');
% 
% figure(6);hold on; view(3); grid on; axis([-0.5 1 -0.5 0.5 -0.2 1]); title('scene');
% 
% figure(7); imagesc(time_to_collision(:,:,150)); colorbar; title('time to collision');
% 
