close all
clear all
clc
format compact

import gtsam.*
import gpmp2.*

%% Environment
t_start_moving = 0;
v_or_t_end = true;

% v_or_t_end_value = [0.2,-0.2];
v_or_t_end_value = [0.2,-0.2];
starting_pos = [0.40, 0.6];
obs_size = [0.60, 0.80];

env = movingEnvironment(0,v_or_t_end,v_or_t_end_value, starting_pos, obs_size);
dataset = env.queryEnv(0);

% % % plot sdf
% figure(1)
% plotSignedDistanceField2D(dataset.field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field');
% hold off;
% pause(0.1);

dataset_1 = env.queryEnv(1);

%% BlockMatcher
% block_size_x = 35;
% block_size_y = 35;
% 
% blkMatcher = vision.BlockMatcher('ReferenceFrameSource','Input port',...
%                             'BlockSize', [block_size_x,block_size_y]);
% blkMatcher.OutputValue = 'Horizontal and vertical components in complex form';
% 
% motion = blkMatcher(dataset.map,dataset_1.map);
% halphablend = vision.AlphaBlender;
% img12 = halphablend(dataset.map,dataset_1.map);
% 
% 
% 
% [X,Y] = meshgrid(1:block_size_x:size(dataset.map,2),1:block_size_y:size(dataset_1.map,1));         
% imshow(img12)
% hold on
% quiver(X(:),Y(:),real(motion(:)),imag(motion(:)),0)
% hold off


%% opticalFlow
% 
% h = figure;
% movegui(h);
% hViewPanel = uipanel(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
% hPlot = axes(hViewPanel);
% 
% flow = opticalFlow;
% 
% for i = 0:0.2:5 
%     dataset = env.queryEnv(i);
%     
%     if i > 0
%         flow = opticalFlow(1-dataset.map, 1-last_step.map);
%         imshow(1-dataset.map)
%         hold on
%         plot(flow,'DecimationFactor',[5 5],'ScaleFactor',2,'Parent',hPlot);
%         hold off
%     end
%     last_step = dataset;
%     pause(0.2)
% end

%% opticalFlowFarneback
% opticFlow = opticalFlowFarneback;
opticFlow = opticalFlowHS;
opticFlow2 = opticalFlowHS;
% % % plot sdf
% figure(1)
% plotSignedDistanceField2D(dataset.field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field');
% hold off;
% pause(0.1);

h = figure;
movegui(h);
hViewPanel = uipanel(h,'Position',[0 0 1 1],'Title','Plot of Optical Flow Vectors');
hPlot = axes(hViewPanel);

delta_t = 1;
for i = 0:delta_t:2 
    dataset = env.queryEnv(i);
%     subplot(1,2,1);
%     ax = gca;
    imshow(1-dataset.map);
    hold on;
    centroid = regionprops(dataset.map).Centroid;
    scatter(centroid(1), centroid(2), 'r');
    if i>0
        px_velocity = centroid-last_centroid/delta_t;
%         subplot(1,2,2);
        hold on
        I = imshow(imtranslate(1-dataset.map, delta_t*px_velocity,'FillValues',255));    end
    hold off
    last_centroid = centroid;
    pause(1)
end


% for i = 0:1:2 
%     dataset = env.queryEnv(i);
%     flow = estimateFlow(opticFlow,dataset.map);
%     subplot(1,2,1);
%     ax = gca;
%     imshow(1-dataset.map)
%     centroid = regionprops(dataset.map).Centroid;
%     hold on
%     scatter(centroid(1), centroid(2), 'r');
%     plot(flow,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',ax);
% %     plot(flow, 'Parent',hPlot);
%     hold off
%     if i>=1
%         old_dataset = env.queryEnv(i-1);
%         old_flow = estimateFlow(opticFlow2,old_dataset.map);
% 
%         subplot(1,2,2);
%         mean_vx = max(max(old_flow.Vx));
%         mean_vy = min(min(old_flow.Vy));
%         imshow(imtranslate(1-old_dataset.map, [mean_vx, mean_vy],'FillValues',255))
%     end
%     pause(1)
% end
% 
% 


% figure(2)
% imshow(1-dataset.map)
% hold on
% plot(flow,'DecimationFactor',[5 5],'ScaleFactor',10,'Parent',hPlot);
% 
% figure(3)
% [x, y] = meshgrid(1:size(dataset.map,2), 1:size(dataset.map,1));
% % generate synthetic test data, for experimenting
% % vx = 0.1*y;   % an arbitrary flow field, in this case
% % vy = 0.1*x;   % representing shear
% % compute the warped image - the subtractions are because we're specifying
% % where in the original image each pixel in the new image comes from
% D = interp2(double(dataset.map), x-flow.Vx, y-flow.Vy);
% % display the result
% imshow(D, []);


%% To translate the image

% imshow(imtranslate(1-dataset.map, [50, 50],'FillValues',255))

