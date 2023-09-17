import open3d as o3d
import numpy as np

from segmentation import ransac
from clustering import dbscan

def extract_clusters(cluster_cloud, labels, max_points=500):
    """
    This function extract the different clusters from the given master cluster pcd
    using the labels parameter

    :param cluster_cloud: a pcd object with different clusters
    :param labels: a list which contains the cluster label of all points from cluster_cloud
    :param max_points: the allowed maximum points in a cluster 
    :return clusters: a dict object with labels as keys and the cluster points as values
    """
    ## Make clusters
    clusters = dict()
    for i, label in enumerate(labels):
        if label not in clusters:
            clusters[label] = list()
        clusters[label].append(cluster_cloud.points[i])

    ## Remove the cluster with -1 label (outliers)
    _ = clusters.pop(-1)

    ## Filtering out clusters with a max points
    clusters = {label: points for label, points in clusters.items() if len(points) <= max_points}
    return clusters

def boundingbox(pcd_list, clusters):
    """
    This function creates the axis-aligned bounding boxes for the different clusters

    :param pcd_list: a list of pcd objects used for visualization
    :param clusters: a dict with labels as keys and cluster points as values
    :return pcd_list: a list of pcd objects appended with bounding boxes 
    """
    for points in clusters.values():
        cluster = o3d.geometry.PointCloud()
        cluster.points = o3d.utility.Vector3dVector(points)
        aabb = cluster.get_axis_aligned_bounding_box()
        aabb.color = (0.9, 0.9, 0.9)
        pcd_list.append(aabb)
    return pcd_list

if __name__ == "__main__":
    ## Read the point cloud file
    pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

    ## Segmentation (Ground vs No Ground)
    inliers_cloud, outliers_cloud = ransac(pcd, 0.35, 3, 100)

    ## Clustering (No ground into cluster)
    cluster_cloud, labels = dbscan(outliers_cloud, 0.33, 10)

    ## Extract clusters from cluster_cloud
    clusters = extract_clusters(cluster_cloud, labels, 1500)
    
    ## Bounding box for the clusters
    #aabb_boxes = list()
    aabb_boxes = [inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)), cluster_cloud]
    aabb_boxes = boundingbox(aabb_boxes, clusters)

    ## Visualize the point cloud 
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                       outliers_cloud.paint_uniform_color((0.1, 0.3, 0.4))])
    o3d.visualization.draw_geometries([inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                       cluster_cloud])
    o3d.visualization.draw_geometries(aabb_boxes)