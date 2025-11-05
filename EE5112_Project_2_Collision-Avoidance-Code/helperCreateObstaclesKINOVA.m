%% Construct voxel obstacle representation for STOMP planner
% Define the workspace (the voxel world limits). Depending on the robot's reach range.
Env_size = [-1.2, -1.0, -0.1; 2.4, 2.0, 1.4];  % [xmin, ymin, zmin] for 1st row, xyz-lengths for 2nd row
voxel_size = [0.02, 0.02, 0.02];  % unit: m
% Binary map: all free space initially, 
binary_world = zeros(Env_size(2, 1) / voxel_size(1), Env_size(2, 2) / voxel_size(2), Env_size(2, 3) / voxel_size(3));
% binary_world_offset = Env_size(1, :)./ voxel_size;
%% XYZ metric representation (in meter) for each voxel 
% !!!!Watch out for the useage of meshgrid: 
% [X,Y,Z] = meshgrid(x,y,z) returns 3-D grid coordinates defined by the 
% vectors x, y, and z. The grid represented by X, Y, and Z has size 
% length(y)-by-length(x)-by-length(z).
% The 3D coordinate is of the center of the voxels
[Xw, Yw, Zw] = meshgrid(Env_size(1, 1) + 0.5 * voxel_size(1) : voxel_size(1) : Env_size(1, 1) + Env_size(2, 1) - 0.5 * voxel_size(1), ...
       Env_size(1, 2) + 0.5 * voxel_size(2) : voxel_size(2) : Env_size(1, 2) + Env_size(2, 2) - 0.5 * voxel_size(2), ...
    Env_size(1, 3) + 0.5 * voxel_size(3) : voxel_size(3) : Env_size(1, 3) + Env_size(2, 3) - 0.5 * voxel_size(3));

%% Static obstacles (more challenging scenario)
obstacleSpecs = struct( ...
    'center', { [0.55 0.25 0.35], [0.2 -0.25 0.22], [0.8 -0.1 0.55] }, ...
    'size',   { [0.16 0.16 0.7], [0.5 0.18 0.44], [0.2 0.6 0.12] } );

world = cell(1, numel(obstacleSpecs));
for i = 1:numel(obstacleSpecs)
    sz = obstacleSpecs(i).size;
    world{i} = collisionBox(sz(1), sz(2), sz(3));
    world{i}.Pose = trvec2tform(obstacleSpecs(i).center);
end

%% voxelize the box obstacles
for i = 1:numel(obstacleSpecs)
    center = obstacleSpecs(i).center;
    sz = obstacleSpecs(i).size;
    metric_box = [center - sz/2;
                  sz];
    voxel_range = [ceil((metric_box(1, :) - Env_size(1,:))./voxel_size); ...
                   ceil((metric_box(1, :) + metric_box(2, :) - Env_size(1,:))./voxel_size)];
    voxel_range = max(voxel_range, 1);
    voxel_range(2,1) = min(voxel_range(2,1), size(binary_world,1));
    voxel_range(2,2) = min(voxel_range(2,2), size(binary_world,2));
    voxel_range(2,3) = min(voxel_range(2,3), size(binary_world,3));
    [xc, yc, zc] = meshgrid(voxel_range(1, 1):voxel_range(2, 1), voxel_range(1, 2):voxel_range(2, 2), voxel_range(1, 3):voxel_range(2, 3));
    binary_world(sub2ind(size(binary_world), xc, yc, zc)) = 1;
end

% % plot the occupied voxel with a marker *
% plot3(xc(:), yc(:), zc(:), '*');
% % Or you can use the volumeViewer() from the Image Processing Toolbox to 
% % display the voxel_world in 3D. 
% volumeViewer(voxel_world);

%% construct signed Euclidean Distance for the voxel world
% Only approximation if the voxel is not a cube
voxel_world_sEDT = prod(voxel_size) ^ (1/3) * sEDT_3d(binary_world);


voxel_world.voxel_size = voxel_size;
voxel_world.voxel = binary_world;
% voxel_world.offset =  binary_world_offset;
voxel_world.world_size = size(binary_world);
voxel_world.Env_size = Env_size; % in metric 
voxel_world.sEDT =  voxel_world_sEDT;





