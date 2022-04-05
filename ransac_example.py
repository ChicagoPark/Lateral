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

#cloud = pcl.load(pointcloudPath)
cloud = pcl.PointCloud(pointcloudPath)

print(cloud)
