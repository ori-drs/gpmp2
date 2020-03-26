close all
clear
clc

import gpmp2.*
import gtsam.*

% Set the velocity of the moving block
shift = [3,5,5]; % Y, X, Z velocity
num_steps = 20;
% env_size = 300;

%% Generate the dataset
dataset = generate3Dmovingdataset('MovingBlock', shift, num_steps);
cell_size = dataset.cell_size;
disp('Generated datasets');

%% Plot the time series of the scene
figure(1);
for i = 0:num_steps
    h = plotCustomOccupancy3D(dataset.fields{i+1}, dataset.origin, dataset.cell_size);
    pause(0.1);
end

%% Visualise the SDF scene with colour
field = dataset.fields{1};
origin = dataset.origin;
epsilon_dist = 0.1;

figure(2);
plotCustomSignedDistanceField3D(field, origin, cell_size, 0, epsilon_dist);
% set(gcf, 'Position',  [1350, 600, 1200, 400])
pause(0.1);

%% Loop through timesteps
figure(3);
set(gcf, 'Position',  [1350, 600, 1200, 700])
pause(0.1);

for i = 1:num_steps
% for i = 1:3
%     tic
    % Propogate the moving object    
    figure(3);
    subplot(1,3,1);
    h = plotCustomSignedDistanceField3D(dataset.fields{i}, dataset.origin, dataset.cell_size);
    
    % Get SDF of the object 

    if i>1
        % Get the change in SDF from last
        D_change = dataset.fields{i} - dataset.fields{i-1};
        figure(3);
        subplot(1,3,2);
        inds_ignore = D_change>0.99*min(min(min(D_change)));
        h = plotCustomSignedDistanceField3D(D_change, dataset.origin, dataset.cell_size);
        title('SDF rate of change');

        % Calculate the time for coillision for each voxel     
        time_to_collision = -dataset.fields{i}./D_change;
        time_to_collision(time_to_collision<0)=1000;
        
        full_time_to_collision = time_to_collision;
%         time_to_collision(inds_ignore)=1000;
        figure(3);
        subplot(1,3,3);   
        plotCustomSignedDistanceField3D(time_to_collision, dataset.origin, dataset.cell_size, 4.8,5);
        title('Time to collision');

    end
    
%     toc 
    %0.2s per time step with figures on
    % Between 0.202811 and 0.127176 seconds without figures
    pause(2)
end

% % Extract collision time for each support state
% traj_collision_mat = zeros(env_size,env_size);
% traj_collision_mat(traj_inds) = full_time_to_collision(traj_inds);
% 
% figure(3);
% set(gcf, 'Position',  [1950, 200, 500, 500])
% h = imagesc(traj_collision_mat);
% colormap(gca, flipud(bone(1000)));
% title('Time to collision');
% xlabel('X/m');
% ylabel('Y/m');
% set(gca, 'YDir','normal')
% 
% 
