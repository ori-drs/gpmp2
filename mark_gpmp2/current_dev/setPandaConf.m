function conf = setPandaConf(conf_name)
    switch conf_name
        case 'ready'
            conf = [0, -0.785, 0, -2.356, 0, 1.57, 0.785]';
        case 'right_ready'
            conf = [-1.57, -0.785, 0, -2.356, 0, 1.57, 0.785]';            
        case 'behind'
            conf = [-3.14, -0.785, 0, -2.356, 0, 1.57, 0.785]';           
        case 'left'
            conf = [1.90, 0.64, 0.01, -1.72, -0.01, 2.36, 1.14]';
            
            
        case 'left_forward_shelf'
            conf = [0.20, 0.63, 0.24, -2.01, -0.28, 2.61, 1.42]';
        case 'right_forward_shelf'
            conf = [-0.65, 0.65, 0.18, -1.94, -0.21, 2.58, 0.46]';
        case 'in_shelf'
            conf = [-2.40, -1.44, 1.11, -1.76, 2.41, 1.78, 2.80]';
        case 'top_shelf'
            conf = [-1.32, 1.42, 1.85, -1.54, -2.61, 2.70, 1.85]';
        case 'left_in_shelf'
            conf = [-0.51, 1.26, 1.80, -1.23, -2.80, 2.20, 2.0]';            
         
            
        case 'table_forward'
            conf = [0, 0.94, -0.07, -1.27, 0.07, 2.21, 0.70]';
        case 'table_right'
            conf = [-0.42, 1.15, 0.18, -0.99, -0.20, 2.13, 0.56]';        
        case 'table_left'
            conf = [0.15, 1.16, 0.22, -0.99, -0.25, 2.12, 1.16]';
        case 'pickup_right'
            conf = [-1.64, 0.70, 0.08, -1.79, -0.08, 2.49, 0.83]';
        case 'pickup_left'
            conf = [1.58, 0.67, 0.04, -1.84, -0.04, 2.52, 0.87]';
        case 'table_forward_horz'
            conf = [0.13, 1.22, -0.12, -1.02, -2.85, 2.51, 0.62]';
        case 'table_left_horz'
            conf = [-0.33, 1.46, -0.42, -0.56, -2.85, 2.59, 0.89]';
        case 'table_right_horz'
            conf = [0.33, 1.46, -0.42, -0.56, -2.85, 2.59, 0.89]';

            
    end
end
