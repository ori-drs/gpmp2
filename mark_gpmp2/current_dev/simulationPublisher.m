function pub = simulationPublisher(node, obstacle)
    switch obstacle    
        case 'person'
            pub = ros.Publisher(node,'/start_moving_person','std_msgs/Float32');    
        case 'cylinder'
            pub = ros.Publisher(node,'/start_moving_panda_cylinder','std_msgs/Float32');    
        case 'hsrb'
            pub = ros.Publisher(node,'/start_moving_hsrb','std_msgs/Float32');
    end

end