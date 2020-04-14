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


%% Loop through timesteps
predict_ahead_steps = 1;
for i = 1:env_size

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
        

        if i>1+predict_ahead_steps
            subplot(2,3,3);
            h = imagesc(sdf_prediction{1});
            colormap(gca, parula(1000));
            title('SDF Prediction');
            xlabel('X/m');
            ylabel('Y/m');
            colorbar;
            set(gca, 'YDir','normal')
            
            subplot(2,3,4);
            h = imagesc(sdf_prediction{1} - D);
            colormap(gca, parula(1000));
            title('SDF Difference');
            xlabel('X/m');
            ylabel('Y/m');
            colorbar;
            set(gca, 'YDir','normal')
        end
   
        % Make prediction for next SDF
        if i<=predict_ahead_steps+1
            sdf_prediction{i-1} = D + D_change * predict_ahead_steps;
        else
            sdf_prediction = sdf_prediction(1,2:end);
            sdf_prediction{predict_ahead_steps} = D + D_change * predict_ahead_steps;
        end
        

    end
    last_D = D;

    pause(0.01)
end
