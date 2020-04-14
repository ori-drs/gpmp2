classdef movingEnvironment3D < handle
    %MOVINGENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        t_start_moving
        v_or_t_end
        v_or_t_end_value
        block_starting_pos
        dataset
        block_velocity
        t_end_moving
        block_end_pos
    end
    
    methods
        function obj = movingEnvironment3D(t_start_moving, v_or_t_end, v_or_t_end_value, block_starting_pos, obs_size)
            %MOVINGENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here            
            obj.t_start_moving = t_start_moving;
            obj.v_or_t_end = v_or_t_end;
            obj.v_or_t_end_value = v_or_t_end_value;

            obj.block_starting_pos = block_starting_pos;
            obj.dataset.obs_size = obs_size;

            % Parameters for block trajectory
            if v_or_t_end == true % object velocity given
                obj.block_velocity = v_or_t_end_value;        
            elseif v_or_t_end == false % t_end given
                obj.t_end_moving = v_or_t_end_value;
                obj.block_end_pos = [2.50, 1.50, 1.50];
                obj.block_velocity = (block_end_pos - block_starting_pos)/(t_end_moving-t_start_moving);
            end
            
            % params
            obj.dataset.cols = 300;
            obj.dataset.rows = 300;
            obj.dataset.z = 300;
            obj.dataset.origin_x = -1;
            obj.dataset.origin_y = -1;
            obj.dataset.origin_z = -1;
            obj.dataset.cell_size = 0.01;
            obj.dataset.origin_point3 = gtsam.Point3(obj.dataset.origin_x, ...
                                                    obj.dataset.origin_y, ...
                                                    obj.dataset.origin_z);       
            
        end
        
        
%         function add_block(t_start_moving, v_or_t_end, v_or_t_end_value, block_starting_pos, obs_size)
%         
%         
%         end
%         
        function updateMap(obj, query_t)
            obj.dataset.map = zeros(obj.dataset.rows, ...
                                    obj.dataset.cols, ...
                                    obj.dataset.z);
            
            if query_t<= obj.t_start_moving
               t_moved = 0;
            else
               t_moved = query_t - obj.t_start_moving;
            end
            
            % Calculate object positions - position is start plus v * t
            x = obj.block_starting_pos(1) + t_moved * obj.block_velocity(1);
            y = obj.block_starting_pos(2) + t_moved * obj.block_velocity(2);
            z = obj.block_starting_pos(3) + t_moved * obj.block_velocity(3);

            if obj.v_or_t_end == false && query_t<= obj.v_or_t_end_value
                x = obj.block_end_pos(1);
                y = obj.block_end_pos(2);
                z = obj.block_end_pos(3);
            end

            %  Add each obstacle
            obs_pos_to_add = round([x - obj.dataset.origin_x, ...
                                    -y + obj.dataset.origin_y, ...
                                    z - obj.dataset.origin_z]/obj.dataset.cell_size) ...
                                    + [0, obj.dataset.rows, 0];

            obj.dataset.map = add_obstacle(obs_pos_to_add, ...
                                            round(obj.dataset.obs_size/obj.dataset.cell_size), ...
                                            obj.dataset.map);
                                
            % Flip the map before calculating SDFs                                    
            obj.dataset.map = flip(obj.dataset.map);   
            
            obj.dataset.obs_pose = [x,y,z];
            
        end
        
        function val = getDataset(obj)
            val =  obj.dataset;
        end

        function val = queryEnv(obj, query_t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % map
            updateMap(obj, query_t);
            
%             obj.dataset.map = zeros(obj.dataset.rows, ...
%                                     obj.dataset.cols, ...
%                                     obj.dataset.z);
%             
%             if query_t<= obj.t_start_moving
%                t_moved = 0;
%             else
%                t_moved = query_t - obj.t_start_moving;
%             end
%             
%             % Calculate object positions - position is start plus v * t
%             x = obj.block_starting_pos(1) + t_moved * obj.block_velocity(1);
%             y = obj.block_starting_pos(2) + t_moved * obj.block_velocity(2);
%             z = obj.block_starting_pos(3) + t_moved * obj.block_velocity(3);
% 
%             if obj.v_or_t_end == false && query_t<= obj.v_or_t_end_value
%                 x = obj.block_end_pos(1);
%                 y = obj.block_end_pos(2);
%                 z = obj.block_end_pos(3);
%             end
% 
%             %  Add each obstacle
%             obs_pos_to_add = round([x - obj.dataset.origin_x, ...
%                                     -y + obj.dataset.origin_y, ...
%                                     z - obj.dataset.origin_z]/obj.dataset.cell_size) ...
%                                     + [0, obj.dataset.rows, 0];
% 
%             obj.dataset.map = add_obstacle(obs_pos_to_add, ...
%                                             round(obj.dataset.obs_size/obj.dataset.cell_size), ...
%                                             obj.dataset.map);
%         
%                                         
%                                         
%             % Flip the map before calculating SDFs                                    
%             obj.dataset.map = flip(obj.dataset.map);
            
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField3D(obj.dataset.map, ...
                                                            obj.dataset.cell_size);            
            
            % init sdf
            obj.dataset.sdf = gpmp2.SignedDistanceField(obj.dataset.origin_point3, ...
                                                obj.dataset.cell_size, ...
                                                size(obj.dataset.field, 1), ...
                                                size(obj.dataset.field, 2), ...
                                                size(obj.dataset.field, 3));
            for z = 1:size(obj.dataset.field, 3)
                obj.dataset.sdf.initFieldData(z-1, obj.dataset.field(:,:,z)');
            end

            
%             obj.dataset.obs_pose = [x,y,z];
            
            val = obj.dataset;
        end
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks)

half_width = floor((obj_size(1)-1)/2);
half_height = floor((obj_size(2)-1)/2);
half_depth = floor((obj_size(3)-1)/2);

% occupency grid
X = position(1)-half_width : position(1)+half_width;
Y = position(2)-half_height : position(2)+half_height;
Z = position(3)-half_depth : position(3)+half_depth;

y_inds_in_range = Y>=1 & Y<= size(map,1);
Y = Y(y_inds_in_range);

x_inds_in_range = X>=1 & X<= size(map,3);
X = X(x_inds_in_range);

z_inds_in_range = Z>=1 & Z<= size(map,3);
Z = Z(z_inds_in_range);


% occupency grid
map(Y, X, Z) = ones(size(Y, 2), size(X, 2), size(Z, 2)); 

end