function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% Obstacle cost following Eq. (13) of Kalakrishnan et al., ICRA 2011
try
    % keep indices inside the voxel map
    idx = max(idx, 1);
    idx(:,1) = min(idx(:,1), world_size(1));
    idx(:,2) = min(idx(:,2), world_size(2));
    idx(:,3) = min(idx(:,3), world_size(3));

    linear_idx = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
    distance_to_obstacle = voxel_world_sEDT(linear_idx);

    % clearance between sphere surface and obstacle
    clearance = distance_to_obstacle - radius(:);
    penetration = max(0, safety_margin - clearance);

    cost_array = (penetration.^2) .* (abs(vel) + 1e-3);
    cost = sum(cost_array);
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end
