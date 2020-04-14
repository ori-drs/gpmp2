function coll_cost = getCollisionCost(inputArg1,inputArg2)
%GETCOLLISIONCOST Summary of this function goes here
%   Detailed explanation goes here

  coll_cost = 0;
  
  ObstaclePlanarSDFFactorArm(...
            key_pos, arm, dataset.sdfs{i+1}, cost_sigma, epsilon_dist)
        
  OBS_FACTOR obs_factor = OBS_FACTOR(Symbol('x', 0), robot, sdf, setting.cost_sigma, 0);
  
  for i=0:result.size()/2
    coll_cost += (obs_factor.evaluateError(result.at<typename ROBOT::Pose>(Symbol('x', i)))).sum();
  end

end

