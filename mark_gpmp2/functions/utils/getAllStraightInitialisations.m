function all_init_values = getAllStraightInitialisations(start_conf, end_conf, total_time_step, delta_t)
%GETALLSTRAIGHTINITIALISATIONS Summary of this function goes here

    all_init_values = [];
    
    dims = size(start_conf, 1);
    
    init_values = gtsam.Values;
    
    % For 
    for j = 1:2^dims      
        for l = 1:2
        if start_conf(1) == 0 && j>1
        	break
        end
            
        for k= 1:dims

            if start_conf(2) == 0 && k>1
                break
            end
            
            init_values = getStraightLineInitialisation(s_conf, g_conf, total_time_step, delta_t);
            all_init_values = [all_init_values, init_values]; 
        end
    end
end

test_start = [1,2,3,4] % 4 dof
test_end = [5,6,7,8]

dims = size(test_start,2);

% For each dimension
possible_start_per_dim = cell(dims);
possible_end_per_dim = cell(dims);

for i = 1:dims
    
end
    