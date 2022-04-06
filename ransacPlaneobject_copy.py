from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

import pcl

import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import csv

sn = int(sys.argv[1]) if len(sys.argv)>1 else 7 #default 7
name = '%06d'%sn # 6 digit zeropadding
pointcloudPath = f'../../../../dataset/training/velodyne/{name}.pcd'


cloud = pcl.load(pointcloudPath)
segmenter = cloud.make_segmenter_normals(ksearch=100)
segmenter.set_optimize_coefficients(True)
segmenter.set_model_type(pcl.SACMODEL_NORMAL_PLANE)  #pcl_sac_model_plane
segmenter.set_normal_distance_weight(0.1)
segmenter.set_method_type(pcl.SAC_RANSAC) #pcl_sac_ransac
segmenter.set_max_iterations(100)
segmenter.set_distance_threshold(0.3) #0.03)  #max_distance
indices, coefficients = segmenter.segment()

inliers = cloud.extract(indices, negative=False)
outliers = cloud.extract(indices, negative=True)

pcl.save(inliers, '/home/kaai/chicago_ws/src/first_pkg/Lateral/inlier.pcd', format=None, binary = True)

pa = inliers.to_array()
print(type(pa))
pa.tofile('inlier.bin')
