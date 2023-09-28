# 3D Object Detection
In this GitHub repository, our primary goal is to access point cloud data from the Kitti dataset and apply various data processing operations, including downsampling, segmentation, and clustering. Ultimately, the project's objective is to employ bounding box techniques to detect objects within the point cloud.

The project specifically focuses on segmenting the ground and detecting 3D objects in point cloud data from the KITTI dataset using the built-in functions of the Open3D library. The final output can be viewed in the following video:

![[Final Output](../document/3d_object_detection.gif)][https://youtu.be/FezeqxAs1CI]

## About the Dataset
For this project, we utilize the KITTI dataset. To obtain this dataset, simply run the following commands from your project directory:

```shell
mkdir data && cd data/
wget https://point-clouds-data.s3.us-west-2.amazonaws.com/KITTI_PCD.zip && unzip KITTI_PCD.zip
rm KITTI_PCD.zip
```

You can preview the point cloud data below:

![Point Cloud Image](./document/distance_point_cloud.jpg)

Various point cloud processing operations using the Open3D library are applied to this point cloud data.

## Downsampling
Voxel downsampling involves employing a regular voxel grid to create a uniformly downsampled point cloud from the input data. In this example, a voxel size of 0.2 is used. Before downsampling, there are 124,473 points, but after downsampling, the number of points reduces to 45,699. The resulting image appears as follows:

![Downsampled Point Cloud](./document/downsampled_point_cloud.jpg)

## Plane Segmentation
Open3D also supports the segmentation of geometric primitives from point clouds using RANSAC. The Open3D function returns the plane equation as (a, b, c, d) such that for each point (x, y, z) on the plane, it satisfies the equation ax + by + cz + d = 0. For this image, the plane equation is -0.00x + 0.03y + 1.00z + 1.79 = 0. This plane equation separates the point cloud data into two parts: inliers (ground data) and outliers (no ground data).

![Plane Segmentation](./document/plane_segmentation.jpg)

## DBSCAN Clustering
To group local point cloud clusters together, especially in the context of a lidar point cloud, clustering algorithms are employed. Open3D implements DBSCAN, a density-based clustering algorithm. When applied to the no ground data from the previous step, it results in the following output.

![DBSCAN Clustering](./document/dbscan_clustering.jpg)

## 3D Bounding Box
The PointCloud geometry type in Open3D includes bounding volumes similar to other geometry types. Currently, Open3D supports AxisAlignedBoundingBox and OrientedBoundingBox, which can also be used for geometry cropping. Principal component analysis (PCA) is used to identify these bounding boxes. When applied to clustered data from the previous step, focusing on clusters with fewer than 1,500 points, it yields the following output:

![3D Bounding Box](./document/3d_bounding_box.jpg)

## Future Plans
While the current approach involves some manual intervention, we have plans to enhance the process through the following strategies:

- KD Tree clustering
- Employing deep learning for plane segmentation
- Utilizing deep learning for clustering

### Credits
This project was inspired by and guided by Jeremy Cohen's course at Think Autonomous.
