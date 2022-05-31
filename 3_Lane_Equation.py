import numpy as np

# create the lane list
leftx =[]
lefty =[]
rightx =[]
righty =[]

leftFile = np.loadtxt('../../CSV_Communication/2_matched_left.txt')
rightFile = np.loadtxt('../../CSV_Communication/2_matched_right.txt')

for i in leftFile:
	leftx.append(i[1])
	lefty.append(i[0])
for i in rightFile:
	rightx.append(i[1])
	righty.append(i[0])

# Pick the largest x value from the left and right first and convert it to type int

max_distance = 0
for i in leftx:
    if max_distance < i:
        max_distance = int(i)
    
for j in rightx:
    if max_distance < j:
        max_distance = int(j)

# Plot quadratic function
        
left_fit = np.polyfit(leftx, lefty, 2) 
right_fit = np.polyfit(rightx, righty, 2)

# The x and y values were defined based on the lidar coordinate axis.
left_fitx = []
left_fity = []
right_fitx = []
right_fity = []

# Obtain the coordinate value for the left(right) equation in 1m increments until the maximum value with a for loop and get the coordinate value for the right
# for left lane
for i in range(max_distance+1):
    left_fitx.append(i)
    left_fity.append(left_fit[0] * i ** 2 + left_fit[1] * i + left_fit[2])
# for right lane
for j in range(max_distance+1):
    right_fitx.append(j)
    right_fity.append(right_fit[0] * j ** 2 + right_fit[1] * j + right_fit[2])
'''
print(left_fitx)
print(left_fity)

print(right_fitx)
print(right_fity)
'''


# 1. put the left x coordinate
print("float left_lane_equat_point_x[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fitx) - 1:
        print(left_fitx[i],'};')
        break
    print(left_fitx[i], ',', end='')
    
# 2. put left y coordinate
print("float left_lane_equat_point_y[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fity) - 1:
        print(left_fity[i],'};')
        break
    print(left_fity[i], ',', end='')
    
        
# 3. put right x coordinate
print("float right_lane_equat_point_x[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fitx) - 1:
        print(right_fitx[i],'};')
        break
    print(right_fitx[i], ',', end='')
    
        
# 4. put right y coordinate
print("float right_lane_equat_point_y[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fity) - 1:
        print(right_fity[i],'};')
        break
    print(right_fity[i], ',', end='')
    
#make left lane txt file
with open('../../CSV_Communication/left_lane.txt', 'w') as left_lane_file:
    for i in range(len(left_fitx)):
        left_lane_file.write(f"{-left_fity[i]} {left_fitx[i]}\n")

#make right lane txt file
with open('../../CSV_Communication/right_lane.txt', 'w') as right_lane_file:
    for i in range(len(right_fitx)):
        right_lane_file.write(f"{-right_fity[i]} {right_fitx[i]}\n")

