classdef objectTrackerPredictor < handle
    %OBJECTTRACKERPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        current_t
        map_size
        static_map
        delta_t
        
        obs_px_velocity
        px_ind_list
        last_centroids
        num_obs
    end
    
    methods
        function obj = objectTrackerPredictor(map_size)
            %OBJECTTRACKERPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.map_size = map_size;
            obj.static_map = zeros(map_size);
            obj.num_obs = 0;
        end
        
        function obj = update(obj, current_t, new_map)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Save new values
            obj.delta_t = current_t - obj.current_t;
            obj.current_t = current_t;
            
            % Calculate the centroids
            conn_comps = bwconncomp(new_map);
            regions = regionprops(conn_comps,'Centroid');
            num_obs = conn_comps.NumObjects;

            obs_centroids = zeros(num_obs, 3);
            obj.obs_px_velocity = zeros(num_obs, 3);

            % For each object
            for j = 1:num_obs
                % Get centroid
                centroid = regions(j).Centroid;
                obs_centroids(j, :) = centroid;

                if obj.num_obs>0
                    % Matching with closest point in last frame
                    [~, min_ind] = min(sum((centroid - obj.last_centroids).^2,2));
                    obj.obs_px_velocity(j,:) = (centroid - obj.last_centroids(min_ind,:))./obj.delta_t;
                end
            end
            
            % Save the centroid locations
            obj.last_centroids = obs_centroids;
            obj.px_ind_list = conn_comps.PixelIdxList;
            obj.num_obs = num_obs;

        end
        
        function predicted_map = predict(obj, t)
            %METHOD2 Summary of this method goes here
            %   Detailed explanation goes here
            predicted_map = obj.static_map;
            
            for j = 1:obj.num_obs
                predicted_occupancy_inds = getPredictedOccupancyInds(t - obj.current_t,...
                                                                    obj.obs_px_velocity(j,:),...
                                                                    obj.map_size,...
                                                                    obj.px_ind_list{j});
                predicted_map(predicted_occupancy_inds) = 1;                                   
            end

            
        end
    end
end


function px_inds = getPredictedOccupancyInds(time, px_velocity, dataset_size, px_list)

    px_velocity_mod = [px_velocity(2),px_velocity(1),px_velocity(3)];
    
    [row,col,z] = ind2sub(dataset_size,px_list);
    obj_coords = horzcat(row,col,z);
    predicted_coords = obj_coords + (px_velocity_mod*time);
    
    % Remove the coords out of the workspace
    keep_mask = all(predicted_coords >= [1,1,1]  & predicted_coords <= dataset_size, 2); % any row that is out of bounds
    predicted_coords = predicted_coords(keep_mask, :);
    
    px_inds = sub2ind(dataset_size,predicted_coords(:,1)', predicted_coords(:,2)', predicted_coords(:,3)');
end
