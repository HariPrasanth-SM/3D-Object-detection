import open3d as o3d

from segmentation import ransac
from clustering import dbscan
from boundingbox import extract_clusters, boundingbox
from visualization import visualize_pcd

def pipeline(pcd, fig_name, open3d_viz=False):
    """
    This function does the entire 3d bounding box estimation for the given point cloud file
    A complete package of all other source codes

    :param pcd: the input pcd object 
    :param fig_name: a string, the figure name to save the processed pcd output
    :param open3d_viz: a bool, to visualise the output in open3d
    :return fig: the plotly fig object after all processing
    """
    ## Segment ground vs no ground
    inlier_cloud, outlier_cloud = ransac(pcd, distance_threshold=0.3, 
                                         ransac_n=3, num_iterations=100)
  
    ## Clustering no ground points
    cluster_cloud, labels = dbscan(outlier_cloud, eps=0.5, min_points=10)

    ## EXtracting clusters
    clusters = extract_clusters(cluster_cloud, labels=labels, max_points=1500)
    
    ## Creating 3d bounding boxes
    aabb_boxes = [inlier_cloud.paint_uniform_color((0.2, 0.1, 0.2)), cluster_cloud]
    aabb_boxes = boundingbox(aabb_boxes, clusters)

    ## Open3d Visualization
    if open3d_viz == True:
        o3d.visualization.draw_geometries([pcd])
        o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                           outlier_cloud.paint_uniform_color((0.1, 0.3, 0.3))])
        o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color((0.2, 0.1, 0.1)),
                                           cluster_cloud])
        o3d.visualization.draw_geometries(aabb_boxes)

    ## Plotly visualization
    fig = visualize_pcd(aabb_boxes, show=False, save=fig_name)
    
    return fig

if __name__ == "__main__":
    open3d_viz = True # Flag to visualize in open3d
    plotly_viz = True # Flag to vizualise & save in Plotly

    ## Read pcd file
    pcd = o3d.io.read_point_cloud("../data/UDACITY/000000.pcd")

    ## 3d bounding box pipeline
    fig = pipeline(pcd, "test")
    fig.show()

    
