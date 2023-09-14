import open3d as o3d

voxel_size = 0.05

# Read point cloud data
pcd = o3d.io.read_point_cloud("../data/PLAYGROUND/turtlebot-pointcloud 2.ply")
print(f"Points before downsampling: {len(pcd.points)}")

# Downsampling the point cloud 
downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)
print(f"Points after downsampling: {len(downpcd.points)}")

# Voxelization
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(downpcd, voxel_size=voxel_size) 

# Visualise the point cloud
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([downpcd])
o3d.visualization.draw_geometries([voxel_grid])

