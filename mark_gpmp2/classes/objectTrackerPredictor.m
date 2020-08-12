 classdef objectTrackerPredictor < handle
    %OBJECTTRACKERPREDICTOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        current_t
        map_size
        static_map
        use_static_map
        delta_t
        cell_size
        origin_point3
        
        obs_px_velocity
        obs_coord_velocity
        px_ind_list
        last_centroids
        num_obs
        object_sdfs
        association_thresh
        obj_sdf_map
        obj_counter
        epsilon
        static_sdf_field
        
        
        
        % Timing params
        init_static_field_time
        calc_obj_sdfs_time
        sdf_coord_track_time
    end
    
    methods
        function obj = objectTrackerPredictor(map_size, static_map, epsilon, cell_size, origin_point3)
            %OBJECTTRACKERPREDICTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.map_size = map_size;
            obj.epsilon = ceil(epsilon/cell_size);
            obj.num_obs = 0;
            obj.origin_point3 = origin_point3;
            obj.object_sdfs = {};
            obj.association_thresh = 4000; % TODO - give a more robust threshold
            obj.obj_counter = 0;
            obj.obj_sdf_map = containers.Map('KeyType','uint32','ValueType','uint32');
            
            if nargin > 1
                obj.static_map = static_map;
                obj.use_static_map = true;
            else
                obj.static_map = zeros(map_size);
                obj.use_static_map = false;
            end
            
            obj.cell_size = cell_size;
            
            object_tic = tic;
            obj.update_static_sdf_field(); % calc and sets the static sdf field initially
            obj.init_static_field_time = toc(object_tic);
            
            obj.calc_obj_sdfs_time = 0;
            obj.sdf_coord_track_time = 0;
        end
        
        function obj = update(obj, current_t, latest_map)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            % Save new values
            obj.delta_t = current_t - obj.current_t;
            obj.current_t = current_t;
            
            % Subtract the static map
            new_map = flip(latest_map) - obj.static_map;
            
            % Calculate the centroids
            conn_comps = bwconncomp(new_map);
            regions = regionprops(conn_comps,'Centroid');
            num_obs = conn_comps.NumObjects;

            obs_centroids = zeros(num_obs, 3);
            obj.obs_px_velocity = zeros(num_obs, 3);

            % For each object found
            for j = 1:num_obs
                
                % Get centroid
                centroid = round(regions(j).Centroid); % note this is in [col,row,z]
                obs_centroids(j, :) = centroid;
                
                if obj.num_obs>0
                    % Matching with closest point in last frame
                    [min_dist, min_ind] = min(sum((centroid - obj.last_centroids).^2,2));
                    ind_closest_obj_sdf = obj.obj_sdf_map(min_ind);
                    % If min_dist to a previous centroid is too large, add
                    % new entry
                    if min_dist < obj.association_thresh
                        px_vel = (centroid - obj.last_centroids(min_ind,:))./obj.delta_t;
                        obj.obs_px_velocity(j,:) = px_vel;
                        
                        update_tic = tic;
                        obj_coord_vel = [px_vel(2), px_vel(1), px_vel(3)];  
                        obj.obs_coord_velocity(j,:) = obj_coord_vel;
                        obj.obj_sdf_map(j) = ind_closest_obj_sdf;
                        obj.object_sdfs{ind_closest_obj_sdf}.min_coord =  obj.object_sdfs{ind_closest_obj_sdf}.min_coord ... 
                                                                                + obj.delta_t * obj_coord_vel; % TODO - use min_coord to centroid and use latest centroid
                        obj.sdf_coord_track_time = obj.sdf_coord_track_time + toc(update_tic);    
                        
                    %Update the new location of thepixel list   
                    else
                        obj.obs_px_velocity(j,:) = [0,0,0];
                        update_tic = tic;
                        obj.object_sdfs{obj.obj_counter+1} = obj.find_sdf_object(conn_comps.PixelIdxList{j});
                        obj.calc_obj_sdfs_time = obj.calc_obj_sdfs_time + toc(update_tic);
                        obj.obj_counter = obj.obj_counter + 1; % New object seen
                        obj.obj_sdf_map(j) = obj.obj_counter;    
                    end
                % First observation
                else
                    update_tic = tic;
                    obj.object_sdfs{end+1} = obj.find_sdf_object(conn_comps.PixelIdxList{j});
                    obj.calc_obj_sdfs_time = obj.calc_obj_sdfs_time + toc(update_tic);
                    obj.obj_counter = obj.obj_counter + 1; % New object seen
                    obj.obj_sdf_map(obj.obj_counter) = j;
                end
      
            end
            
            % Save the centroid locations
            obj.last_centroids = obs_centroids;
            obj.px_ind_list = conn_comps.PixelIdxList;
            
            % Update number of objects found
            obj.num_obs = num_obs;

        end
        
        
        function object_field_data =  find_sdf_object(obj, px_list)

            [row,col,z] = ind2sub(obj.map_size,px_list);
            min_coord = [min(row), min(col), min(z)];
            max_coord = [max(row), max(col), max(z)]; 
            
            new_sdf_size = max_coord - min_coord + [1,1,1]*((2*obj.epsilon)+1); % plus 1 needed    

            min_coord = min_coord - obj.epsilon;
    
            row = row - min_coord(1) + 1;
            col = col - min_coord(2) + 1;
            z = z - min_coord(3) + 1;
            occ_inds = sub2ind(new_sdf_size,row, col, z);
            object_occ_grid = zeros(new_sdf_size);
            object_occ_grid(occ_inds) = 1;
            object_field_data.field  = gpmp2.signedDistanceField3D(object_occ_grid, obj.cell_size);            
            object_field_data.min_coord = min_coord;
            object_field_data.size = new_sdf_size;
        end
       
        
        function predicted_sdf = predict_object_locations(obj, t)
            
            forward_t = t - obj.current_t;
            
            predicted_sdf = obj.static_sdf_field;

            for j = 1:obj.num_obs
                
                obj_sdf_ind = obj.obj_sdf_map(j);
                obj_sdf_data = obj.object_sdfs{obj_sdf_ind};
                sdf_size = obj_sdf_data.size;
                lower_coord = obj_sdf_data.min_coord;
                
%                 if any(isnan(lower_coord))
%                     continue
%                 end
                
                %  If the velocity data has been acquired, do prediction
                if j <= length(obj.obs_coord_velocity)
                    predicted_coord = round(lower_coord + ...
                                                (obj.obs_coord_velocity(j,:)*forward_t));
                else
                    predicted_coord = lower_coord;
                end
                                
                rows_range  = predicted_coord(1):predicted_coord(1) + sdf_size(1) - 1;
                cols_range  = predicted_coord(2):predicted_coord(2) + sdf_size(2) - 1;
                z_range     = predicted_coord(3):predicted_coord(3) + sdf_size(3) - 1;
                
                flip_rows_range = obj.map_size(1) + 1 - rows_range;
                
                % Find inds out range to remove
                valid_row_mask  = flip_rows_range    >= 1 & flip_rows_range   <= obj.map_size(1);
                valid_col_mask  = cols_range    >= 1 & cols_range   <= obj.map_size(2);
                valid_z_mask    = z_range       >= 1 & z_range      <= obj.map_size(3);
                
                flip_rows_range  = flip_rows_range(valid_row_mask);
                cols_range  = cols_range(valid_col_mask);
                z_range     = z_range(valid_z_mask);
    
                new_sdf_rows_range  = 1:sdf_size(1);
                new_sdf_rows_range  = new_sdf_rows_range(valid_row_mask);
                new_sdf_cols_range  = 1:sdf_size(2);
                new_sdf_cols_range  = new_sdf_cols_range(valid_col_mask);
                new_sdf_z_range     = 1:sdf_size(3);
                new_sdf_z_range     = new_sdf_z_range(valid_z_mask);
                

                static_sdf_slice    = predicted_sdf(flip_rows_range, cols_range, z_range);
                new_sdf_slice       = obj_sdf_data.field(new_sdf_rows_range, new_sdf_cols_range, new_sdf_z_range);
                
                
                predicted_sdf(flip_rows_range, cols_range , z_range) = min(static_sdf_slice, new_sdf_slice);
            end
            
            predicted_sdf = permute(predicted_sdf, [2 1 3]);
            
        end
        
        function obj = update_static_sdf_field(obj)
            obj.static_sdf_field = gpmp2.signedDistanceField3D(flip(obj.static_map), obj.cell_size);  
            permute(obj.static_sdf_field, [2 1 3]); % Only for equivalent timing comparison
        end
                    
       function predicted_map = predict(obj, t)

            predicted_map = obj.static_map;
            
            for j = 1:obj.num_obs
                predicted_occupancy_inds = getPredictedOccupancyInds(t - obj.current_t,...
                                                                    obj.obs_px_velocity(j,:),...
                                                                    obj.map_size,...
                                                                    obj.px_ind_list{j});
                predicted_map(predicted_occupancy_inds) = 1;                                   
            end  
       end

        function field = predict_field(obj, t)

            predicted_map = obj.static_map;
            
            for j = 1:obj.num_obs
                predicted_occupancy_inds = getPredictedOccupancyInds(t - obj.current_t,...
                                                                    obj.obs_px_velocity(j,:),...
                                                                    obj.map_size,...
                                                                    obj.px_ind_list{j});
                predicted_map(predicted_occupancy_inds) = 1;                                   
            end  
           
            predicted_map  = permute(predicted_map, [2 1 3]);
            
            % Convert map to sdf
            field  = gpmp2.signedDistanceField3D(predicted_map, obj.cell_size);            

        end
        
        function field = predict_sdf(obj, t)

            predicted_map = obj.static_map;
            
            for j = 1:obj.num_obs
                predicted_occupancy_inds = getPredictedOccupancyInds(t - obj.current_t,...
                                                                    obj.obs_px_velocity(j,:),...
                                                                    obj.map_size,...
                                                                    obj.px_ind_list{j});
                predicted_map(predicted_occupancy_inds) = 1;                                   
            end  
           
            predicted_map  = permute(predicted_map, [2 1 3]);
            
            % Convert map to sdf
            field  = gpmp2.signedDistanceField3D(predicted_map, obj.cell_size);            
            
            % init sdf
%             predicted_sdf = gpmp2.SignedDistanceField(obj.origin_point3, ...
%                                                 obj.cell_size, ...
%                                                 size(field, 1), ...
%                                                 size(field, 2), ...
%                                                 size(field, 3));
% 
%             for z = 1:size(field, 3)
%                 predicted_sdf.initFieldData(z-1, field(:,:,z)');
%             end
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
end