function collision_costs = getCollisionErrorVector(graph, obs_factor_inds, result)
%GETCOLLISIONERRORVECTOR Summary of this function goes here
%   Detailed explanation goes here
    num_obs_factors = size(obs_factor_inds,2);
    num_time_steps = max(obs_factor_inds(2,:));
    
    collision_costs = zeros(1, num_time_steps+1); % Assume we aren't in collision at start
    for i = 1 : num_obs_factors
        fact_ind = obs_factor_inds(1,i);
        collision_costs(obs_factor_inds(2,i)+1) = collision_costs(obs_factor_inds(2,i)+1) + graph.at(fact_ind).error(result);
    end
end

