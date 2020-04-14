function positions = confToPositions(arm_model, confs)
%CONFTOPOSITIONS Summary of this function goes here
%   Detailed explanation goes here
    positions = [];
    for i=1:size(confs,2)
        conf = confs(i);               
        positions = [positions;arm_model.forwardKinematicsPosition(conf)];
    end
end

