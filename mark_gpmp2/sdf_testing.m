close all
clear all
clc
format compact

% Nice colormaps: hsv, winter, bone, parula

env_size = 1000;
block_size = env_size/20;
x_start = 2;
y_start = env_size-block_size;
vx = 10;
vy= -20;


fh = figure(1);
set(gcf, 'Position',  [1350, 600, 1200, 700])

%% Trajectory
traj_mat = zeros(env_size,env_size);
for i = 1:env_size
    traj_mat(i,i) = i+500;
end

traj_inds = traj_mat > 0;

% Show the sdf of the trajectory with a dotted line 

% subplot(1,3,1);
% [traj_D,traj_idx] = bwdist(traj_mat);
% h = imagesc(traj_mat);
% colormap(gca, gray(2000));
% xlabel('X/m');
% ylabel('Y/m');
% set(gca, 'YDir','normal')



%% Loop through timesteps
for i = 1:env_size
% for i = 1:3
%     tic
    % Propogate the moving object    
    test_mat = zeros(env_size,env_size);
	dummy_mat = test_mat;
    
    x = x_start + i * vx;
    y = y_start + i * vy;
    
    if x<=env_size-block_size && y <=env_size-block_size && x>=1 && y >=1
        for j=0:block_size
            for k=0:block_size
                dummy_mat(y+j,x+k) = 1;
            end
        end
    else
        break
    end
    figure(1);

    subplot(2,3,1);
    h = imagesc(dummy_mat);
    colormap(gca, gray(2));
    title('Occupancy');
    xlabel('X/m');
    ylabel('Y/m');
    set(gca, 'YDir','normal')

    % Get SDF of the object 
%     figure(1);
    [D,idx] = bwdist(dummy_mat);
    subplot(2,3,2);
    h = imagesc(D);
    colormap(gca, parula(1000));
    title('SDF');
    xlabel('X/m');
    ylabel('Y/m');
    colorbar;

    set(gca, 'YDir','normal')

    if i>1
        % Get the change in SDF from last
%         figure(1);
        D_change = D - last_D;
        inds_ignore = D_change>0.99*min(min(D_change));
%         % Make prediction for next SDF
%         D_prediction 
%         if i>2
            
%         end
        % Plot rate of change
        subplot(2,3,4);
        h = imagesc(D_change);
        colormap(gca, parula(5));
        colorbar;
        title('SDF rate of change');
        xlabel('X/m');
        ylabel('Y/m');
        set(gca, 'YDir','normal')

%         Calculate the time for coillision for each voxel
        subplot(2,3,5);
        
        time_to_collision = -D./D_change;
        time_to_collision(time_to_collision<0)=1000;
        
        full_time_to_collision = time_to_collision;
        time_to_collision(inds_ignore)=1000;

        h = imagesc(time_to_collision);
        colormap(gca, parula(10));
        colorbar;
        title('Time to collision');
        xlabel('X/m');
        ylabel('Y/m');
        set(gca, 'YDir','normal')

    end
    
    last_D = D;
%     toc 
    %0.2s per time step with figures on
    %0.013s per time step without figures on
    pause(0.01)
end

% Extract collision time for each support state
traj_collision_mat = zeros(env_size,env_size);
traj_collision_mat(traj_inds) = full_time_to_collision(traj_inds);

figure(3);
set(gcf, 'Position',  [1950, 200, 500, 500])
h = imagesc(traj_collision_mat);
colormap(gca, flipud(bone(1000)));
title('Time to collision');
xlabel('X/m');
ylabel('Y/m');
set(gca, 'YDir','normal')

