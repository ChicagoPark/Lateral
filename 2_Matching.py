import sys
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import csv
from ransacPlaneobject import *

sn = int(sys.argv[1]) if len(sys.argv)>1 else 7 #default 7
name = '%06d'%sn # 6 digit zeropadding
img = f'../../../../dataset/training/image_2/{name}.png'
binary = f'../../../../dataset/training/velodyne1/{name}.bin'
pcd= f'../../../../dataset/training/velodyne/{name}.pcd'
with open(f'../../../../dataset/training/calib/{name}.txt','r') as f:
    calib = f.readlines()

# read the lane pixel from 1_Lane_2D.py
'''
[About csv file]
line 0: the coefficient of left lane equation
line 1st: the coefficient of right lane equation
line 2nd: leftlaneX
line 3rd: leftlaneY
line 4th: rightlaneX
line 5th: rightlaneY
'''
lane_csv = open("../../CSV_Communication/1_lane.csv")

# get the plane pointcloud data and object pointcloud data
planePCDarray, objectPCDarray = ransacPlaneobject(pcd)


csvreader = csv.reader(lane_csv)
rows = []
for row in csvreader:
    rows.append(row)
# get the lane lists from csv
leftlaneX = rows[2]
leftlaneY = rows[3]
rightlaneX = rows[4]
rightlaneY = rows[5]

leftlane3D = []
leftlane3Dx = []
leftlane3Dy = []
leftlane3Dz = []
rightlane3D = []
rightlane3Dx = []
rightlane3Dy = []
rightlane3Dz = []

# P2 (3 x 4) for left eye
P2 = np.matrix([float(x) for x in calib[2].strip('\n').split(' ')[1:]]).reshape(3,4)
R0_rect = np.matrix([float(x) for x in calib[4].strip('\n').split(' ')[1:]]).reshape(3,3)
# Add a 1 in bottom-right, reshape to 4 x 4
R0_rect = np.insert(R0_rect,3,values=[0,0,0],axis=0)
R0_rect = np.insert(R0_rect,3,values=[0,0,0,1],axis=1)
Tr_velo_to_cam = np.matrix([float(x) for x in calib[5].strip('\n').split(' ')[1:]]).reshape(3,4)
Tr_velo_to_cam = np.insert(Tr_velo_to_cam,3,values=[0,0,0,1],axis=0)

# read raw data from binary
#scan = np.fromfile(binary, dtype=np.float32).reshape((-1,4))
scan = planePCDarray
points = scan[:, 0:3] # lidar xyz (front, left, up)

print(points.shape)
# TODO: use fov filter? 
velo = np.insert(points,3,1,axis=1).T
print(velo.shape)
velo = np.delete(velo,np.where(velo[0,:]<0),axis=1)
cam = P2 * R0_rect * Tr_velo_to_cam * velo
#cam = np.delete(cam,np.where(cam[2,:]<0)[1],axis=1)
new_cam = np.transpose(cam)
new_velo = np.transpose(velo)


new_veloList = list(np.squeeze(np.asarray(new_velo)))
new_camList = list(np.squeeze(np.asarray(new_cam)))
print("YOUNGIL")

index = 0
# loop the lidar points
for i in range(len(new_veloList)):
    '''
    veloList[0]: 3D x value
    veloList[1]: 3D y value
    veloList[2]: 3D z value
    '''
    veloList = list(np.squeeze(np.asarray(new_velo[i]).reshape(-1)))
    
    onecamList = list(np.squeeze(np.asarray(new_camList[i]).reshape(-1)))
    
    #print(f"1st: {onecamList[0]} / 2nd: {onecamList[1]} / 3rd: {onecamList[2]}")

    project2Dx= onecamList[0]/onecamList[2]
    project2Dy = onecamList[1]/onecamList[2]

    #print(f"first: {project2Dx}/ second: {project2Dy}")

    index += 1
    # consider just the front view
    if veloList[0] >=0:
        # loop the left lane pixels
        for j in range(len(leftlaneX)):
            if int(project2Dx)==int(leftlaneX[j]) and int(project2Dy)==int(leftlaneY[j]):
                leftlane3D.append([veloList[0], veloList[1], veloList[2]])

        # loop the right lane pixels
        for j in range(len(rightlaneX)):
             if int(project2Dx)==int(rightlaneX[j]) and int(project2Dy)==int(rightlaneY[j]):
                rightlane3D.append([veloList[0], veloList[1], veloList[2]])


leftlane3D = sorted(leftlane3D)
rightlane3D = sorted(rightlane3D)
print(leftlane3D)
print(rightlane3D)

# for c language matching code --------------------------------->>
print("For C language Matching Code")
print(f"count_left = {len(leftlane3D)};\nfloat left_lane_x[] = ", end='');print('{',end='')
for i in range(len(leftlane3D)-1):
    print(f"{leftlane3D[i][0]},",end='')
print(leftlane3D[-1][0],end='');print('};')

print(f"float left_lane_y[] = ", end='');print('{',end='')
for i in range(len(leftlane3D)-1):
    print(f"{leftlane3D[i][1]},",end='')
print(leftlane3D[-1][1],end='');print('};')

print(f"float left_lane_z[] = ", end='');print('{',end='')
for i in range(len(leftlane3D)-1):
    print(f"{leftlane3D[i][2]},",end='')
print(leftlane3D[-1][2],end='');print('};')

print(f"count_right = {len(rightlane3D)};\nfloat right_lane_x[] = ", end='');print('{',end='')
for i in range(len(rightlane3D)-1):
    print(f"{rightlane3D[i][0]},",end='')
print(rightlane3D[-1][0],end='');print('};')

print(f"float right_lane_y[] = ", end='');print('{',end='')
for i in range(len(rightlane3D)-1):
    print(f"{rightlane3D[i][1]},",end='')
print(rightlane3D[-1][1],end='');print('};')

print(f"float right_lane_z[] = ", end='');print('{',end='')
for i in range(len(rightlane3D)-1):
    print(f"{rightlane3D[i][2]},",end='')
print(rightlane3D[-1][2],end='');print('};')
# for c language matching code ---------------------------------<<



#make left lane txt file
with open('../../CSV_Communication/2_matched_left.txt', 'w') as left_lane_file:
    for i in range(len(leftlane3D)):
        left_lane_file.write(f"{leftlane3D[i][1]} {leftlane3D[i][0]}\n")

#make right lane txt file
with open('../../CSV_Communication/2_matched_right.txt', 'w') as right_lane_file:
    for i in range(len(rightlane3D)):
        right_lane_file.write(f"{rightlane3D[i][1]} {rightlane3D[i][0]}\n")



# get u,v,z
cam[:2] /= cam[2,:]
# do projection staff
plt.figure(figsize=(12,5),dpi=96,tight_layout=True)
png = mpimg.imread(img)
IMG_H,IMG_W,_ = png.shape

# restrict canvas in range
plt.axis([0,IMG_W,IMG_H,0])
plt.imshow(png)
# filter point out of canvas
u,v,z = cam
u_out = np.logical_or(u<0, u>IMG_W)
v_out = np.logical_or(v<0, v>IMG_H)
outlier = np.logical_or(u_out, v_out)
cam = np.delete(cam,np.where(outlier),axis=1)
# generate color map from depth
u,v,z = cam
plt.scatter([u],[v],c=[z],cmap='rainbow_r',alpha=0.5,s=2)
plt.title(name)
plt.savefig(f'../../../../dataset/projected_output/{name}.png',bbox_inches='tight')
plt.show()
