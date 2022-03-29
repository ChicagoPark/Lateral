import numpy as np

# 3D 상 Point 따기

'''
Put the value from previous code
'''
#leftx = [6.692,7.556,8.194,9.036,9.427,10.279,13.227,13.799,14.37,14.97,19.949,21.408,23.078,24.556,26.85,36.293]
#lefty = [1.462,1.466,1.463,1.455,1.449,1.437,1.455,1.432,1.47,1.442,1.482,1.517,1.547,1.552,1.595,1.769]
#rightx = [6.995,7.23,7.508,8.665,9.082,9.88,11.788,12.23,13.391,15.161,15.602,16.408,18.582,26.081,27.98,36.211,45.941]
#righty = [-1.886,-1.869,-1.866,-1.843,-1.819,-1.78,-1.732,-1.732,-1.695,-1.624,-1.561,-1.515,-1.488,-1.404,-1.359,-1.309,-1.063]

leftx = [7.385,7.669,9.253,11.566,12.073,15.646,16.476,17.435,21.102,22.819,26.647,28.843,35.311]
lefty = [2.311,2.318,2.315,2.316,2.307,2.33,2.305,2.299,2.365,2.324,2.383,2.365,2.445]
rightx = [6.63,7.313,7.588,7.887,8.219,8.555,9.074,9.826,10.327,10.986,12.594,23.96,30.847,38.824]
righty = [-2.167,-2.196,-2.178,-2.16,-2.172,-2.151,-2.139,-2.115,-2.081,-2.09,-2.112,-2.051,-2.036,-2.077]


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

