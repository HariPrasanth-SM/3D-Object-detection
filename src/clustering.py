import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

from segmentation import ransac

## Read point cloud file
pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

## RANSAC Segmentation
inlier_cloud, outlier_cloud = ransac(pcd)

## DBSCAN Clustering 
labels = np.asarray(outlier_cloud.cluster_dbscan(eps=0.5, min_points=10, print_progress=True))
max_label = labels.max()
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])


## Visualize the point cloud
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color((0.1, 0.4, 0.4)), outlier_cloud])