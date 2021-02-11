classdef curvedEnvironment < handle

    properties 
        dataset
        objects = {};
        fixed_scene = {}
        sub
        obs_size
        
        obstacle
        obstacle_id
        obstacle_offset
        scene
    end
    
    methods
        function env = curvedEnvironment(env_size, resolution, origin)

            if nargin > 2
                env.dataset.origin_x = origin(1);
                env.dataset.origin_y = origin(2);
                env.dataset.origin_z = origin(3);
            else
                env.dataset.origin_x = -1;
                env.dataset.origin_y = -1;
                env.dataset.origin_z = -1;
            end
            
            pause(2);
            

            env.obs_size = [0.55, 0.55, 1.2];
            env.obstacle_offset = 0.6;            
            
            env.dataset.cols = env_size;
            env.dataset.rows = env_size;
            env.dataset.z = env_size;


            env.dataset.cell_size = resolution;
            env.dataset.origin_point3 = gtsam.Point3(env.dataset.origin_x, ...
                                                    env.dataset.origin_y, ...
                                                    env.dataset.origin_z);       
                                                
            env.dataset.static_map = zeros(env.dataset.rows, ...
                                    env.dataset.cols, ...
                                    env.dataset.z);    
                                
            env.dataset.map = env.dataset.static_map;                                
            env.dataset.obs_poses = {};
            
            env.add_table_static_scene();
        end
        
        function add_table_static_scene(env)
         
            % Table
            stat_obs{1} = {[-0.31, 0, 0.2], [0.80, 1.2 , 0.4]};

%             stat_obs{2} = {[1.05, 0, 0.2], [0.80, 1.2 , 0.4]};

            origin = [env.dataset.origin_x, ...
                       env.dataset.origin_y, ...
                       env.dataset.origin_z];
                   
            for i = 1:size(stat_obs, 2)
                obj_cell_coords = round((stat_obs{i}{1} - origin) / env.dataset.cell_size);
                obj_cell_size = round(stat_obs{i}{2} / env.dataset.cell_size);

                env.dataset.static_map = add_obstacle(obj_cell_coords, ...
                                obj_cell_size, ...
                                env.dataset.static_map); 
            end
            env.dataset.static_map = flip(env.dataset.static_map);          

        end
          
   
        function obs_pos_to_add = calculateObjPosition(env, block_pos) 
            
            % Calculate object positions - position is start plus v * t
            x = block_pos(1);
            y = block_pos(2);
            z = block_pos(3) + env.obstacle_offset;

            %  Add each obstacle
            obs_pos_to_add = round([x - env.dataset.origin_x, ...
                                    -y + env.dataset.origin_y, ...
                                    z - env.dataset.origin_z]/env.dataset.cell_size) ...
                                    + [1, env.dataset.rows,1];

            env.dataset.obs_poses{1} = [x, y, z];  
            
        end
        
        function updateMap(env, block_pos)
                    
            env.dataset.map = env.dataset.static_map;
                        
            obs_pos_to_add = env.calculateObjPosition(block_pos);

            env.dataset.map = add_obstacle(obs_pos_to_add, ...
                                            round(env.obs_size/env.dataset.cell_size), ...
                                            env.dataset.map);
                                        
            % Flip the map before calculating SDFs                                    
            env.dataset.map = flip(env.dataset.map);          
 
        end
        
        function val = getDataset(obj)
            val =  obj.dataset;
        end

        function val = getSDF(obj)
            
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField3D(permute(obj.dataset.map, [2 1 3]), ...
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

            val = obj.dataset.sdf;
        end
        
        function resetMap(obj)
            env.dataset.map = flip(env.dataset.static_map);  
        end
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks)

half_width = ceil(obj_size(1)/2);
half_height = ceil(obj_size(2)/2);
half_depth = ceil(obj_size(3)/2);

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


% occupancy grid
map(Y, X, Z) = ones(size(Y, 2), size(X, 2), size(Z, 2)); 

end