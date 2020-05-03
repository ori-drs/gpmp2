classdef movingEnvironment3D < handle
    %MOVINGENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
%         t_start_moving
%         v_or_t_end
%         v_or_t_end_value
%         block_starting_pos
        dataset
%         block_velocity
%         t_end_moving
%         block_end_pos
        
        objects = {};
        fixed_scene = {}
    end
    
    methods
        function env = movingEnvironment3D()
            %MOVINGENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here           
            
            % params
            env.dataset.cols = 300;
            env.dataset.rows = 300;
            env.dataset.z = 300;
            env.dataset.origin_x = -1;
            env.dataset.origin_y = -1;
            env.dataset.origin_z = -1;
            env.dataset.cell_size = 0.01;
            env.dataset.origin_point3 = gtsam.Point3(env.dataset.origin_x, ...
                                                    env.dataset.origin_y, ...
                                                    env.dataset.origin_z);       
                                                
            env.dataset.static_map = zeros(env.dataset.rows, ...
                                    env.dataset.cols, ...
                                    env.dataset.z);   
%             env.add_static_scene;          
%             env.add_static_scene();
            
%             add_object(t_start_moving, v_or_t_end, v_or_t_end_value, block_starting_pos, obs_size)                                    
           
        end
        
        function add_static_scene(env)
         
            stat_obs{1} = {[170 220 130], [140, 60, 5]};
            stat_obs{2} = {[105 195 90], [10, 10, 80]};
            stat_obs{3} = {[235 195 90], [10, 10, 80]};
            stat_obs{4} = {[105 245 90], [10, 10, 80]};
            stat_obs{5} = {[235 245 90], [10, 10, 80]};
            
            stat_obs{6} = {[250 190 145], [60, 5, 190]};
            stat_obs{7} = {[250 90 145], [60, 5, 190]};
            
            stat_obs{8} = {[200 190 145], [40, 5, 190]};

            stat_obs{9} = {[250 140 240], [60, 100, 5]};
            stat_obs{10} = {[250 140 190], [60, 100, 5]};
            stat_obs{11} = {[250 140 140], [60, 100, 5]};
            stat_obs{12} = {[250 140 90], [60, 100, 5]};
                        

            origin = [env.dataset.origin_x, ...
                      env.dataset.origin_y, ...
                      env.dataset.origin_z]/env.dataset.cell_size;
                  
            for i = 1:size(stat_obs, 2)
                env.dataset.static_map = add_obstacle(stat_obs{i}{1} -[50,50,50], ...
                                stat_obs{i}{2}, ...
                                env.dataset.static_map); 
            end
            env.dataset.static_map = flip(env.dataset.static_map);          

        end
        
        function add_object(env, t_start_moving, t_end_moving, block_vel, block_starting_pos, obs_size)
            obj.t_start_moving = t_start_moving;
            obj.t_end_moving = t_end_moving;

            obj.block_starting_pos = block_starting_pos;
            obj.obs_size = obs_size;
            
            obj.block_velocity = block_vel;        

            % Parameters for block trajectory
%             if t_end_moving == 0 % object velocity given
%             elseif t_end_moving ~= 0 % t_end given
%                 obj.t_end_moving = t_end_moving;
%                 obj.block_end_pos = [2.50, 1.50, 1.50];
%                 obj.block_velocity = (obj.block_end_pos - obj.block_starting_pos)/(obj.t_end_moving-obj.t_start_moving);
%             end
            
            env.objects{end+1} = obj;
        
        end
%       
        function [obs_pos_to_add, obs_pos] = calculateObjPosition(env, t, object) 
            if t<= object.t_start_moving
                   t_moved = 0;
            else
                   t_moved = t - object.t_start_moving;
                   if object.t_end_moving ~= 0 && t_moved >  object.t_end_moving 
                        t_moved = object.t_end_moving;
                   end
                   
            end

            % Calculate object positions - position is start plus v * t
            x = object.block_starting_pos(1) + t_moved * object.block_velocity(1);
            y = object.block_starting_pos(2) + t_moved * object.block_velocity(2);
            z = object.block_starting_pos(3) + t_moved * object.block_velocity(3);
% 
%             if object.v_or_t_end == false && t<= object.v_or_t_end_value
%                 x = object.block_end_pos(1);
%                 y = object.block_end_pos(2);
%                 z = object.block_end_pos(3);
%             end

            %  Add each obstacle
            obs_pos_to_add = round([x - env.dataset.origin_x, ...
                                    -y + env.dataset.origin_y, ...
                                    z - env.dataset.origin_z]/env.dataset.cell_size) ...
                                    + [1, env.dataset.rows,1];
            obs_pos = [x, y, z];        
        end
        
        function updateMap(env, query_t)
                    
            env.dataset.map = env.dataset.static_map;
            
            % Add all the obstacles
            for i = 1:size(env.objects, 2)
                
                [obs_pos_to_add, obs_pos] = env.calculateObjPosition(query_t, env.objects{i});
             
                env.dataset.map = add_obstacle(obs_pos_to_add, ...
                                                round(env.objects{i}.obs_size/env.dataset.cell_size), ...
                                                env.dataset.map);
                                            
                env.dataset.obs_poses{i} = obs_pos;  
            end
            
            % Flip the map before calculating SDFs                                    
            env.dataset.map = flip(env.dataset.map);          
 
        end
        
        function val = getDataset(obj)
            val =  obj.dataset;
        end

        function val = queryEnv(obj, query_t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            % map
            updateMap(obj, query_t);
            
%                         env.dataset.map = flip(env.dataset.map);          

                       
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField3D(permute(obj.dataset.map, [2 1 3]), ...
                                                obj.dataset.cell_size); 
                                                        
%             obj.dataset.field  = gpmp2.signedDistanceField3D(permute(obj.dataset.map, [2 1 3]), ...
%                                                             obj.dataset.cell_size);            
%             
            % init sdf
            obj.dataset.sdf = gpmp2.SignedDistanceField(obj.dataset.origin_point3, ...
                                                obj.dataset.cell_size, ...
                                                size(obj.dataset.field, 1), ...
                                                size(obj.dataset.field, 2), ...
                                                size(obj.dataset.field, 3));
            for z = 1:size(obj.dataset.field, 3)
                obj.dataset.sdf.initFieldData(z-1, obj.dataset.field(:,:,z)');
            end

            val = obj.dataset;
        end
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks)

half_width = floor(obj_size(1)/2);
half_height = floor(obj_size(2)/2);
half_depth = floor(obj_size(3)/2);

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