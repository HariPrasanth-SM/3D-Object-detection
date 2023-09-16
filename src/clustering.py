import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from segmentation import ransac

def dbscan(pcd, eps=0.02, min_points=10):
    """
    This function uses the DBSCAN algorithm to identify different clusters in the given pcd file
    and color code the clusters, and return as a new point cloud file

    :param pcd: input point cloud data
    :param eps: the distance to the neighbours in the cluster
    :param min_points: minimum number of points required to form a cluster
    :return cluster_cloud: point cloud file with different color-coded clusters
    :return labels: the labels of every points from the point cloud
    """
    ## DBSCAN clustering
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    
    ## Color coding for different clusters
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0

    ## Create a new pcd object to store clustering outputs
    cluster_cloud = o3d.geometry.PointCloud()
    cluster_cloud.points = o3d.utility.Vector3dVector(pcd.points)
    cluster_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])

    return cluster_cloud, labels

if __name__=="__main__":
    ## Read point cloud file
    pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

    ## RANSAC Segmentation
    inlier_cloud, outlier_cloud = ransac(pcd)

    ## DBSCAN Clustering 
    cluster_cloud, labels = dbscan(outlier_cloud, 0.5, 10)

    ## Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color((0.1, 0.4, 0.4)), cluster_cloud])