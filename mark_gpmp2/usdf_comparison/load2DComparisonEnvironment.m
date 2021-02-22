classdef load2DComparisonEnvironment < handle

    properties 
        dataset    
        objects = {};
        fixed_scene = {}
    end
    
    methods
        function env = load2DComparisonEnvironment(env_size, resolution, origin)

            env.dataset.origin_x = origin(1);
            env.dataset.origin_y = origin(2);

                      
            env.dataset.cols = env_size;
            env.dataset.rows = env_size;


            env.dataset.cell_size = resolution;
            env.dataset.origin_point2 = gtsam.Point2(env.dataset.origin_x, ...
                                                    env.dataset.origin_y);       
                                    
            env.dataset.static_map = zeros(env.dataset.rows, ...
                                    env.dataset.cols);                            
       
        end
        
        function add_object(env, block_pos, obs_size)

            obj.block_pos = block_pos;
            obj.obs_size = obs_size;
            env.objects{end+1} = obj;
        
        end
       
        function obs_pos_to_add = calculateObjPosition(env, object) 

            % Calculate object positions - position is start plus v * t
            x = object.block_pos(1);
            y = object.block_pos(2);

            %  Add each obstacle
            obs_pos_to_add = round([y - env.dataset.origin_y, ...
                                    x - env.dataset.origin_x]/env.dataset.cell_size);
        end
        
        function updateMap(env)
                    
            env.dataset.map = env.dataset.static_map;
            
            % Add all the obstacles
            for i = 1:size(env.objects, 2)
                
                obs_pos_to_add = env.calculateObjPosition(env.objects{i});
             
                env.dataset.map = add_obstacle(obs_pos_to_add, ...
                                                round(env.objects{i}.obs_size/env.dataset.cell_size), ...
                                                env.dataset.map);
                                            
            end      
 
        end
        
        function val = getDataset(obj)
            val =  obj.dataset;
        end

        function val = queryEnv(obj)

            updateMap(obj);
            
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField2D(permute(obj.dataset.map, [2,1]), ...
                                                obj.dataset.cell_size); 
                                                        
            % init sdf
            obj.dataset.sdf = gpmp2.PlanarSDF(obj.dataset.origin_point2, ...
                                              obj.dataset.cell_size, ...
                                              obj.dataset.field);

            val = obj.dataset;
        end
        
        
        function val = getSDF(obj)

            updateMap(obj);
            
            % signed distance field
            obj.dataset.field  = gpmp2.signedDistanceField2D(permute(obj.dataset.map, [2,1]), ...
                                                obj.dataset.cell_size); 
                                                        
            % init sdf
            obj.dataset.sdf = gpmp2.PlanarSDF(obj.dataset.origin_point2, ...
                                              obj.dataset.cell_size, ...
                                              obj.dataset.field);


            val = obj.dataset.sdf;
        end

        function val = getUSDF(obj)

            updateMap(obj);
            
            % signed distance field
            obj.dataset.field  = unsignedDistanceField2D(permute(obj.dataset.map, [2 1]), ...
                                                obj.dataset.cell_size); 
                                                        
            % init sdf
            obj.dataset.sdf = gpmp2.PlanarSDF(obj.dataset.origin_point2, ...
                                              obj.dataset.cell_size, ...
                                              obj.dataset.field);


            val = obj.dataset.sdf;
        end
        
        
    end
end

function [map, landmarks] = add_obstacle(position, obj_size, map, landmarks)

half_width = floor(obj_size(1)/2);
half_height = floor(obj_size(2)/2);

% occupency grid
X = position(1)-half_width : position(1)+half_width;
Y = position(2)-half_height : position(2)+half_height;

y_inds_in_range = Y>=1 & Y<= size(map,1);
Y = Y(y_inds_in_range);

x_inds_in_range = X>=1 & X<= size(map,2);
X = X(x_inds_in_range);


% occupancy grid
map(Y, X) = ones(size(Y, 2), size(X, 2)); 

end