import numpy as np

# 3D 상 Point 따기

'''
Put the value from previous code
'''


# generate the lane list
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
with open('../../CSV_Communication/left_lane.txt', 'w') as left_lane_file:
    for i in range(len(left_fitx)):
        left_lane_file.write(f"{-left_fity[i]} {left_fitx[i]}\n")

#make right lane txt file
with open('../../CSV_Communication/right_lane.txt', 'w') as right_lane_file:
    for i in range(len(right_fitx)):
        right_lane_file.write(f"{-right_fity[i]} {right_fitx[i]}\n")

