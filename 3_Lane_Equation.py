import numpy as np

# 3D 상 Point 따기

'''
Put the value from previous code
'''

#leftx = [7.385,7.669,9.253,11.566,12.073,15.646,16.476,17.435,21.102,22.819,26.647,28.843,35.311]
#lefty = [2.311,2.318,2.315,2.316,2.307,2.33,2.305,2.299,2.365,2.324,2.383,2.365,2.445]
#rightx = [6.63,7.313,7.588,7.887,8.219,8.555,9.074,9.826,10.327,10.986,12.594,23.96,30.847,38.824]
#righty = [-2.167,-2.196,-2.178,-2.16,-2.172,-2.151,-2.139,-2.115,-2.081,-2.09,-2.112,-2.051,-2.036,-2.077]
#leftx =[-1.676, -1.673, -1.671, -1.671, -1.671, -1.669, -1.665, -1.665, -1.665, -1.664, -1.663, -1.663, -1.663, -1.661, 2.329, 2.361, 6.437, 7.345, 7.36, 7.649, 7.653, 7.653, 7.67, 7.983, 7.983, 7.997, 7.997, 8.257, 8.295, 8.295, 8.816, 8.844, 8.844, 8.872, 9.223, 9.229, 9.229, 9.583, 9.583, 9.614, 10.061, 10.069, 10.069, 10.666, 10.666, 10.666, 10.681, 11.596, 11.596, 11.605, 11.64, 11.64, 12.081, 12.081, 12.099, 12.128, 12.435, 12.435, 12.439, 12.439, 12.439, 13.101, 13.101, 13.101, 13.64, 13.678, 13.686, 13.686, 14.236, 14.236, 14.24, 14.24, 14.819, 14.832, 14.846, 14.846, 15.653, 15.71, 15.71, 16.481, 16.481, 16.487, 17.434, 17.434, 17.434, 18.573, 18.579, 18.579, 18.587, 18.587, 19.699, 19.699, 19.722, 21.075, 21.102, 22.758, 22.776, 22.776, 22.819, 24.162, 24.162, 24.193, 24.193, 24.225, 26.619, 26.619, 26.647, 26.647, 28.793, 28.843, 28.843, 28.908, 31.783, 31.85, 31.85, 31.943, 35.311, 35.311, 35.388, 35.388]
#lefty =[-1.661, 2.297, 2.324, 2.34, 2.342, 2.353, 2.353, 2.354, 2.356, 2.357, 2.358, 2.358, 2.36, 2.361, 2.361, 2.365, 2.371, 2.371, 2.371, 2.371, 2.377, 2.379, 2.379, 2.38, 2.38, 2.385, 2.385, 2.389, 2.389, 2.389, 2.389, 2.39, 2.391, 2.391, 2.392, 2.392, 2.399, 2.399, 2.4, 2.4, 2.4, 2.401, 2.401, 2.402, 2.402, 2.402, 2.402, 2.403, 2.403, 2.404, 2.407, 2.407, 2.408, 2.408, 2.408, 2.408, 2.409, 2.409, 2.41, 2.41, 2.412, 2.415, 2.415, 2.417, 2.417, 2.42, 2.421, 2.422, 2.427, 2.427, 2.428, 2.431, 2.433, 2.433, 2.437, 2.437, 2.441, 2.441, 2.443, 2.444, 2.444, 2.446, 2.451, 2.456, 2.457, 2.462, 2.464, 2.464, 2.496, 6.437, 6.45, 6.457, 6.733, 6.741, 6.741, 7.093, 7.093, 7.093, 7.094, 7.098, 7.117, 7.343, 7.343, 26.619, 26.619, 26.647, 26.647, 28.793, 28.843, 28.843, 28.908, 31.783, 31.85, 31.85, 31.943, 35.311, 35.311, 35.388, 35.388, 39.693]
#rightx =[-1.451, 2.313, 2.329, 2.335, 2.343, 2.343, 2.356, 2.364, 2.366, 2.37, 2.37, 2.37, 2.386, 2.386, 2.395, 6.393, 6.648, 6.964, 6.964, 7.327, 7.345, 7.602, 7.604, 7.913, 7.913, 8.226, 8.244, 8.56, 8.563, 8.563, 9.085, 9.489, 9.507, 9.845, 9.849, 10.328, 11, 12.061, 12.078, 12.594, 13.003, 13.003, 13.662, 13.673, 14.254, 14.254, 14.257, 14.841, 14.841, 15.449, 15.458, 15.458, 16.297]
#righty =[-2.163, -2.158, -2.156, -2.149, -2.146, -2.143, -2.143, -2.143, -2.141, -2.141, -2.123, -2.123, -2.111, -2.11, -2.11, -2.11, -2.104, -2.087, -2.08, -2.077, -2.074, -2.057, -2.056, -2.056, -2.052, -2.052, -2.051, -2.047, -2.042, -2.042, -2.038, -2.034, -2.034, -2.029, -2.024, -2.024, -2.004, -1.998, -1.993, -1.977, -1.972, -1.967, -1.967, -1.966, -1.966, -1.949, -1.937, -1.934, -1.907, -1.894, -1.875, -1.874, -1.866]

# generate the lane list
leftx =[]
lefty =[]
rightx =[]
righty =[]

leftFile = np.loadtxt('/home/kaai/chicago_ws/src/CSV_Communication/2_matched_left.txt')
rightFile = np.loadtxt('/home/kaai/chicago_ws/src/CSV_Communication/2_matched_right.txt')

for i in leftFile:
	leftx.append(i[1])
	lefty.append(i[0])
for i in rightFile:
	rightx.append(i[1])
	righty.append(i[0])

# 좌측 우측 중 최대 x 값을 먼저 뽑고 int 형으로 변환하기

max_distance = 0
for i in leftx:
    if max_distance < i:
        max_distance = int(i)
    
for j in rightx:
    if max_distance < j:
        max_distance = int(j)

# 2차 함수로 플라팅하기
        
left_fit = np.polyfit(leftx, lefty, 2) 
right_fit = np.polyfit(rightx, righty, 2)

# 라이다 좌표축 기준으로 x, y 값을 정의하였다.
left_fitx = []
left_fity = []
right_fitx = []
right_fity = []

# for 반복문으로 최대 값 전까지 1m 단위로 left(right) 방정식 에 대한 좌표값을 받고 right 에 대한 좌표값을 받기
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
# C++ 코드에 넘겨주기


# 1. left x 좌표 넣기
print("float left_lane_equat_point_x[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fitx) - 1:
        print(left_fitx[i],'};')
        break
    print(left_fitx[i], ',', end='')
    
# 2. left y 좌표 넣기
print("float left_lane_equat_point_y[] = {", end='')

for i in range(max_distance + 1):
    if i == len(left_fity) - 1:
        print(left_fity[i],'};')
        break
    print(left_fity[i], ',', end='')
    
        
# 3. right x 좌표 넣기
print("float right_lane_equat_point_x[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fitx) - 1:
        print(right_fitx[i],'};')
        break
    print(right_fitx[i], ',', end='')
    
        
# 4. right y 좌표 넣기
print("float right_lane_equat_point_y[] = {", end='')

for i in range(max_distance+1):
    if i == len(right_fity) - 1:
        print(right_fity[i],'};')
        break
    print(right_fity[i], ',', end='')
    
#make left lane txt file
with open('/home/kaai/chicago_ws/src/CSV_Communication/left_lane.txt', 'w') as left_lane_file:
    for i in range(len(left_fitx)):
        left_lane_file.write(f"{-left_fity[i]} {left_fitx[i]}\n")

#make right lane txt file
with open('/home/kaai/chicago_ws/src/CSV_Communication/right_lane.txt', 'w') as right_lane_file:
    for i in range(len(right_fitx)):
        right_lane_file.write(f"{-right_fity[i]} {right_fitx[i]}\n")

