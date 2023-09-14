import open3d as o3d

def ransac(pcd, distance_threshold=0.33, ransac_n=3, num_iterations=100):
    """
    This function uses the RANSAC algorithm to segment the input point cloud data 
    into ground data (inlier) and no ground data (outlier)

    :param pcd: input point cloud data
    :param distance_threshold: the maximum distance a point can be to be considered as an inlier
    :param ransac_n: number of points that randomly sampled to estimate a plane
    :param num_iterations: how often a random plane is sampled and verified
    :return inlier_cloud: gound point cloud, aka all the points inside of the inlier
    :return outlier_cloud: no ground point cloud, aka the points outside of the inlier    
    """

    ## Segmenting points into ground points and no ground points
    plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
    inlier_cloud = pcd.select_by_index(inliers)
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    ## Segment plane equation
    a, b, c, d = plane_model
    print(f"The plane equation is {a:.2f}x+{b:.2f}y+{c:.2f}z+{d:.2f}=0")
    
    return inlier_cloud, outlier_cloud

## Read point cloud file
pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

inlier_cloud, outlier_cloud = ransac(pcd, 0.35, 3, 100)

## Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color((0.25, 0.1, 0.1)),
                                   outlier_cloud.paint_uniform_color((0.1, 0.3, 0.3))])