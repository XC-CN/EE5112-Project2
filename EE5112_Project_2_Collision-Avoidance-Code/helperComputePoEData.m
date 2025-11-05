function poeData = helperComputePoEData(robot_struct, endEffector)
%HELPERCOMPUTEPOEDATA Pre-compute screw axes and home transforms.
%   This function extracts the spatial screw axes (columns of Slist) and
%   the zero-configuration transforms of each body required by the PoE
%   forward kinematics implementation.

robot_struct.DataFormat = 'column';
homeConfigStruct = robot_struct.homeConfiguration;
nJoints = numel(homeConfigStruct);
homeConfig = zeros(nJoints,1);
for i = 1:nJoints
    homeConfig(i) = homeConfigStruct(i).JointPosition;
end

% Space Jacobian at the home configuration gives the screw axes directly
[Slist, ~] = geometricJacobian(robot_struct, homeConfig, endEffector);

bodyNames = robot_struct.BodyNames;
nBodies = numel(bodyNames);
Mlist = zeros(4,4,nBodies);
for i = 1:nBodies
    Mlist(:,:,i) = getTransform(robot_struct, homeConfig, bodyNames{i});
end

poeData.Slist = Slist;
poeData.Mlist = Mlist;
poeData.bodyNames = bodyNames;
poeData.homeConfig = homeConfig;
