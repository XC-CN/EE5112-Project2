% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize = size(trajProb, 2);
% variable declaration
dtheta = zeros(nJoints, nDiscretize);

%% Iterate over all joints to compute dtheta according to STOMP
for jointIdx = 1:nJoints
    em_joint = em{jointIdx};
    dtheta(jointIdx, :) = sum(trajProb .* em_joint, 1);
end
