import numpy as np

'''
Put the value from previous code
'''
leftx = [3D-POINT]
lefty = [3D-POINT]
rightx = [3D-POINT]
righty = [3D-POINT]


# From left to right, pick the maximum value x first and convert it to int.

max_distance = 0
for i in leftx:
    if max_distance < i:
        max_distance = int(i)
    
for j in rightx:
    if max_distance < j:
        max_distance = int(j)

# get the 2 dimensional equation
left_fit = np.polyfit(leftx, lefty, 2) 
right_fit = np.polyfit(rightx, righty, 2)

left_fitx = []
left_fity = []
right_fitx = []
right_fity = []

# get the corresponding y coordinates about x values which have 1m uniformed gap
# for left lane
for i in range(max_distance+1):
    left_fitx.append(i)
    left_fity.append(left_fit[0] * i ** 2 + left_fit[1] * i + left_fit[2])
# for right lane
for j in range(max_distance+1):
    right_fitx.append(j)
    right_fity.append(right_fit[0] * j ** 2 + right_fit[1] * j + right_fit[2])


# 1. left x coordinates
print("float left_lane_equat_point_x[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fitx) - 1:
        print(left_fitx[i],'};')
        break
    print(left_fitx[i], ',', end='')
    
# 2. left y coordinates
print("float left_lane_equat_point_y[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fity) - 1:
        print(left_fity[i],'};')
        break
    print(left_fity[i], ',', end='')
    
        
# 3. right x coordinates
print("float right_lane_equat_point_x[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fitx) - 1:
        print(right_fitx[i],'};')
        break
    print(right_fitx[i], ',', end='')
    
        
# 4. right y coordinates
print("float right_lane_equat_point_y[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fity) - 1:
        print(right_fity[i],'};')
        break
    print(right_fity[i], ',', end='')
    
