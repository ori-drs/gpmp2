classdef liveEnvironment < handle
    %MOVINGENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        dataset
        objects = {};
        fixed_scene = {}
        sub
        obs_size
    end
    
    methods
        function env = liveEnvironment(node, env_size, resolution, origin)
            %MOVINGENVIRONMENT Construct an instance of this class
            %   Detailed explanation goes here           
            
            % params
            if nargin > 2
                env.dataset.origin_x = origin(1);
                env.dataset.origin_y = origin(2);
                env.dataset.origin_z = origin(3);
            else
                env.dataset.origin_x = -1;
                env.dataset.origin_y = -1;
                env.dataset.origin_z = -1;
            end
                
            env.sub = ros.Subscriber(node,'/gazebo/model_states','gazebo_msgs/ModelStates');
            pause(2);
            
            env.obs_size = [0.15,0.15,0.15];
            
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
        end
        
        function add_table_static_scene(env)
         
            %  Note these are true for 3mx3mx3m at 1cm res          
             stat_obs{1} = {[0, 0, -0.4], [1, 1, 0.08]};
            
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
                
        function add_static_scene(env)
         
            %  Note these are true for 3mx3mx3m at 1cm res          
            stat_obs{1} = {[0.20 0.70 -0.20], [1.40, 0.60, 0.05]};
            
            stat_obs{2} = {[-0.45 0.45 -0.60], [0.10, 0.10, 0.80]};
            
            stat_obs{3} = {[0.85 0.45 -0.60], [0.10, 0.10, 0.80]};
            
            stat_obs{4} = {[-0.45 0.95 -0.60], [0.10, 0.10, 0.80]};
            
            stat_obs{5} = {[0.85 0.95 -0.60], [0.10, 0.10, 0.80]};
            
            stat_obs{6} = {[1.00 0.40 -0.05], [0.60, 0.05, 1.90]};
            
            stat_obs{7} = {[1.00 -0.60 -0.05], [0.60, 0.05, 1.90]};

            stat_obs{8} = {[0.50 0.40 -0.05], [0.40, 0.05, 1.90]};
            
            stat_obs{9} = {[1.00 -0.10 0.90], [0.60, 1.00, 0.05]};

            stat_obs{10} = {[1.00 -0.10 0.40], [0.60, 1.00, 0.05]};

            stat_obs{11} = {[1.00 -0.10 -0.10], [0.60, 1.00, 0.05]};

            stat_obs{12} = {[1.00 -0.10 -0.60], [0.60, 1.00, 0.05]};

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
        
        
        function obs_pos_to_add = calculateObjPosition(env, msg) 
            
            block_pos = msg.Pose(5).Position;

            % Calculate object positions - position is start plus v * t
            x = block_pos.X;
            y = block_pos.Y;
            z = block_pos.Z;

            %  Add each obstacle
            obs_pos_to_add = round([x - env.dataset.origin_x, ...
                                    -y + env.dataset.origin_y, ...
                                    z - env.dataset.origin_z]/env.dataset.cell_size) ...
                                    + [1, env.dataset.rows,1];

            env.dataset.obs_poses{1} = [x, y, z];  
            
        end
        
        function updateMap(env)
                    
            env.dataset.map = env.dataset.static_map;
            
            msg = env.sub.LatestMessage;
            
            if length(msg.Pose) > 4
                obs_pos_to_add = env.calculateObjPosition(msg);

                env.dataset.map = add_obstacle(obs_pos_to_add, ...
                                                round(env.obs_size/env.dataset.cell_size), ...
                                                env.dataset.map);
            end
            
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