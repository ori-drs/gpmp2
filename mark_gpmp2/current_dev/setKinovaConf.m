function conf = setKinovaConf(conf_name)
    switch conf_name
        case 'ready'
            conf = [-1.71, 2.26, 0.78, 0, 0, 0]';
        case 'forward'
            conf = [-1.64, 4.33, 2.77, 0, 0, 0]';
        case 'behind'
            conf = [1.65, 2.48, 0.53, 0, 0, 0]';
        case 'left'
            conf = [-3.24, 4.58, 3.08, 0, 0, 0]';
            
    end
end