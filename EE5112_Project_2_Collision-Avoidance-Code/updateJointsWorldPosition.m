%% forward kinematics
% INPUT: 
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base
%       frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta, poeData)

% Product of Exponentials based forward kinematics implementation
if nargin < 3 || isempty(poeData)
    error('PoE data must be supplied. Call helperComputePoEData first.');
end

Slist = poeData.Slist;
Mlist = poeData.Mlist;
nJoints = length(theta);
if size(Slist,2) ~= nJoints
    error('Mismatch between screw axis list and joint dimension.');
end

T = cell(1, nJoints);
X = zeros(nJoints, 4);
Tprod = eye(4);

for k = 1:nJoints
    expSk = expm(vecToSe3(Slist(:,k)) * theta(k));
    Tprod = Tprod * expSk;
    T_k = Tprod * Mlist(:,:,k);
    T{k} = T_k;
    X(k,:) = (T_k * [0;0;0;1])';
end
    
end
