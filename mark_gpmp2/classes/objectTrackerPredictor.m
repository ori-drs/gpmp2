classdef objectTrackerPredictor < handle
    %OBJECTTRACKERPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        current_t
        map_size
        static_map
        use_static_map
        delta_t
%         cell_size
        
        obs_px_velocity
        px_ind_list
        last_centroids
        num_obs
    end
    
    methods
        function obj = objectTrackerPredictor(map_size, static_map)
            %OBJECTTRACKERPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.map_size = map_size;
            obj.num_obs = 0;
            if nargin == 2
%                 obj.static_map = permute(static_map, [2 1 3]);
                obj.static_map = flip(static_map);
                obj.use_static_map = true;
            else
                obj.static_map = zeros(map_size);
                obj.use_static_map = false;
            end

%             obj.cell_size = cell_size;
        end
        
        function obj = update(obj, current_t, new_map)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Save new values
            obj.delta_t = current_t - obj.current_t;
            obj.current_t = current_t;
            
            % Subtract the static map
            new_map = new_map - obj.static_map;
            
            % Calculate the centroids
            conn_comps = bwconncomp(new_map);
            regions = regionprops(conn_comps,'Centroid');
            num_obs = conn_comps.NumObjects;

            obs_centroids = zeros(num_obs, 3);
            obj.obs_px_velocity = zeros(num_obs, 3);

            % For each object
            for j = 1:num_obs
                % Get centroid
                centroid = round(regions(j).Centroid);
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
        
        
        
        function predicted_sdf = predict_sdf(obj, t, cell_size, origin_point3)
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
            
            % Convert map to sdf
            field  = gpmp2.signedDistanceField3D(permute(predicted_map, [2 1 3]), cell_size);            

            % init sdf
            predicted_sdf = gpmp2.SignedDistanceField(origin_point3, ...
                                                cell_size, ...
                                                size(field, 1), ...
                                                size(field, 2), ...
                                                size(field, 3));

            predicted_sdf.initFieldData(0, field(:,:,1)');
            predicted_sdf.initFieldData(1, field(:,:,2)');
            predicted_sdf.initFieldData(2, field(:,:,3)');

    
        end
        
    end
end


function px_inds = getPredictedOccupancyInds(time, px_velocity, dataset_size, px_list)

    px_velocity_mod = [px_velocity(2),px_velocity(1),px_velocity(3)];
    
    [row,col,z] = ind2sub(dataset_size,px_list);
    obj_coords = horzcat(row,col,z);
    predicted_coords = uint8(obj_coords + (px_velocity_mod*time));
    
    % Remove the coords out of the workspace
    keep_mask = all(predicted_coords >= [1,1,1]  & predicted_coords <= dataset_size, 2); % any row that is out of bounds
    predicted_coords = predicted_coords(keep_mask, :);
    
    px_inds = sub2ind(dataset_size,predicted_coords(:,1)', predicted_coords(:,2)', predicted_coords(:,3)');
%     px_inds = sub2ind(dataset_size,predicted_coords(:,2)', predicted_coords(:,1)', predicted_coords(:,3)');
end