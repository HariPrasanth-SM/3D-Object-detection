import open3d as o3d

## Read point cloud file
pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

## Segmenting grond points and no ground points
plane_model, inliers = pcd.segment_plane(distance_threshold=0.33, ransac_n=3, num_iterations=100)
inlier_points = pcd.select_by_index(inliers) # Ground points
outlier_points = pcd.select_by_index(inliers, invert=True) # No Ground points

## Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([inlier_points.paint_uniform_color((0.25, 0.1, 0.1)),
                                   outlier_points.paint_uniform_color((0.1, 0.3, 0.3))])