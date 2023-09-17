import open3d as o3d
import numpy as np

from segmentation import ransac
from clustering import dbscan

## Read the point cloud file
pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

## Segmentation (Ground vs No Ground)
inliers_cloud, outliers_cloud = ransac(pcd, 0.35, 3, 100)

## Clustering (No ground into cluster)
cluster_cloud, labels = dbscan(outliers_cloud, 0.33, 10)

## Make clusters
clusters = {}
for i, label in enumerate(labels):
    if label not in clusters:
        clusters[label] = list()
    clusters[label].append(outliers_cloud.points[i])

_ = clusters.pop(-1)
clusters = {label: points for label, points in clusters.items() if len(points) <= 300}

#aabb_boxes = list()
aabb_boxes = [inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)), cluster_cloud]
for points in clusters.values():
    cluster = o3d.geometry.PointCloud()
    cluster.points = o3d.utility.Vector3dVector(points)
    aabb = cluster.get_axis_aligned_bounding_box()
    aabb.color = (0.1, 0.1, 0.1)
    aabb_boxes.append(aabb)

## Visualize the point cloud 
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                   outliers_cloud.paint_uniform_color((0.1, 0.3, 0.4))])
o3d.visualization.draw_geometries([inliers_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                   cluster_cloud])
o3d.visualization.draw_geometries(aabb_boxes)