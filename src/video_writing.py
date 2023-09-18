import cv2
import open3d as o3d
from tqdm import tqdm
from glob import glob


from pipeline import pipeline

## Create a video writer object
output_handle = cv2.VideoWriter("../output/object_detection.avi", 
                                cv2.VideoWriter_fourcc(*"XVID"), 10, (2400, 1599))

## Index of PCD files
start_index = 180
stop_index = 230

## Progress bar to indicate the progress
pbar = tqdm(total=(stop_index-start_index), position=0, leave=True)

## Read all point cloud files
point_cloud_files = sorted(glob("../data/KITTI_PCD/*.pcd"))
all_files = [o3d.io.read_point_cloud(point_cloud_files[i]) for i in range(start_index, stop_index)]

## Processing the object detection and writing a video
for i in range(len(all_files)):
    pipeline(all_files[i], str(start_index+i))
    output_handle.write(cv2.imread("../output/"+str(start_index+i)+".jpg"))
    pbar.update(1)

output_handle.release()
