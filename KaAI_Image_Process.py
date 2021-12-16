#!/usr/bin/env python


# Lane Module Beginning >>>>>>>>>>>>>------------------>>>>>>>>>>>>>------------------
import numpy as np, cv2, os
import torch, os, cv2
from model.model import parsingNet
from utils1.common import merge_config
from utils1.dist_utils import dist_print
import math
import torch
import scipy.special, tqdm
import numpy as np
import torchvision.transforms as transforms
from data1.dataset import LaneTestDataset
from data1.constant import culane_row_anchor, tusimple_row_anchor, youngil_row_anchor
from PIL import Image


image_original = cv2.imread("/home/kaai/chicago_ws/src/first_pkg/src/KITTI/image/0000000200.png")

# Lane detection
device = torch.device('cuda' if torch.cuda.is_available() else "cpu")
cls_num_per_lane = 18
net = parsingNet(pretrained=False, backbone='18', cls_dim=(201, cls_num_per_lane, 4),
                 use_aux=False).cuda()  # we dont need auxiliary segmentation in testing
state_dict = torch.load('/home/kaai/lee_ws/src/AB3DMOT/configs/culane_18.pth',
                        map_location='cpu')['model']
compatible_state_dict = {}
for k, v in state_dict.items():
    if 'module.' in k:
        compatible_state_dict[k[7:]] = v
    else:
        compatible_state_dict[k] = v
net.load_state_dict(compatible_state_dict, strict=False)
net.eval()

frame = image_original
# cv2.namedWindow("Youngil")
row_anchor = youngil_row_anchor
img_w, img_h = frame.shape[1], frame.shape[0]
print("img_w, img_h")
print(img_w)
print(img_h)
print("img_w, img_h")
# model_input = torch.from_numpy(frame)
model_input = Image.fromarray(frame)
model_input = transforms.Resize((288, 800))(model_input)
model_input = transforms.ToTensor()(model_input)
model_input = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(model_input)

model_input = model_input.to(device).float()

with torch.no_grad():
    out = net(model_input[None, ...])

col_sample = np.linspace(0, 800 - 1, 200)
col_sample_w = col_sample[1] - col_sample[0]
# out_j has information about each line from the left
out_j = out[0].data.cpu().numpy()
out_j = out_j[:, ::-1, :]
prob = scipy.special.softmax(out_j[:-1, :, :], axis=0)
idx = np.arange(200) + 1
idx = idx.reshape(-1, 1, 1)
loc = np.sum(prob * idx, axis=0)
out_j = np.argmax(out_j, axis=0)
loc[out_j == 200] = 0
out_j = loc
num_lane = out_j.shape[1]
if num_lane <= 2:
    c_c = 50
    d_c = 150
elif num_lane == 3:
    c_c = 0
    d_c = 200
    print("y\no\nu\nn\ng\ni\nl")
else:
    c_c = 0
    d_c = 200

leftx = []
lefty = []
rightx = []
righty = []

left_lane_list = []
for i in range(num_lane):
    point_list_for_curve = []
    if np.sum(out_j[:, i] != 0) > 2:
        for k in range(out_j.shape[0]):
            if out_j[k, i] > 0:
                # ppp has (a, b) format. This means each point location !
                ppp = (int(out_j[k, i] * col_sample_w * img_w / 800) - 1,
                       int(img_h * (row_anchor[cls_num_per_lane - 1 - k] / 288)) - 1)
                if i == 1:
                    # print(ppp[0], '    ', ppp[1])
                    leftx.append(ppp[0])
                    lefty.append(ppp[1])
                    left_lane_list.append(ppp)
                    # print(leftx)
                if i == 2:
                    # print(ppp[0], '    ', ppp[1])
                    rightx.append(ppp[0])
                    righty.append(ppp[1])
                # Drawing lane points
                cv2.circle(frame, ppp, 5, (0, c_c, d_c), -1)  # for plotting 1, 2, 3, 4 lanes as a cv2.circle
                point_list_for_curve.append(ppp)
    point_list_for_curve = np.array(point_list_for_curve, np.int32)
    # cv2.polylines(frame2, [point_list_for_curve], False, (0, c_c, d_c), thickness=7)
#print(left_lane_list)
k = []  # plot 을 하기위해서 담는 좌측 레인의 x, y 좌표
left_lane_all_x = []
left_lane_all_y = []
l = []  # plot 을 하기위해서 담는 우측 레인의 x, y 좌표
right_lane_all_x = []
right_lane_all_y = []

# 2차 함수 구하고, 그래프를 그리는 코드
if len(leftx) != 0 and len(lefty) != 0 and len(rightx) != 0 and len(righty) != 0:
    left_fit = np.polyfit(lefty, leftx, 2)  # to calculate coefficient
    right_fit = np.polyfit(righty, rightx, 2)

    print(f'Left Lane : {left_fit[0]} {left_fit[1]} {left_fit[2]}')
    print(f'Right Lane : {right_fit[0]} {right_fit[1]} {right_fit[2]}')

    # define coefficient of
    if left_fit[1] != 0 and left_fit[2] != 0 and right_fit[0] != 0 and right_fit[1] != 0:
        L = left_fit
        R = right_fit

    # finding point of contact

    a = left_fit[0] - right_fit[0]
    b = left_fit[1] - right_fit[1]
    c = left_fit[2] - right_fit[2]
    A = a

    if a != 0 and b * b - 4 * a * c >= 0:  # case for a != 0
        contact_y = (-b - math.sqrt(b * b - 4 * a * c)) / (2 * a)
        contact_y = int(contact_y)
        contact_y = contact_y + 15
    else:  # case for a == 0. Just finding contact point of lines
        contact_y = -c / b
        contact_y = int(contact_y)
        contact_y = contact_y + 15

    contact_y = abs(contact_y)

    contact_x = left_fit[0] * math.pow(contact_y, 2) + left_fit[1] * contact_y + left_fit[2]

    # ploty = np.linspace(0, frame.shape[0] - 1, frame.shape[0])

    ploty = np.linspace(contact_y, frame.shape[0] - 1,
                        frame.shape[0] - contact_y)  # y좌표에 대해서 상위 1/3 지점 부터 최하단 까지 그래프를 그리기위한 코드

    # ploty = np.linspace(int(frame2.shape[0] * (6 / 11)), frame2.shape[0] - 1,
    #                    int(frame2.shape[0] * (5 / 11)))  # y좌표에 대해서 상위 1/3 지점 부터 최하단 까지 그래프를 그리기위한 코드

    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]  # 방정식에 대한 x 값을 얻어오기 위한 코드

    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    for ii, x in enumerate(left_fitx):
        q = (int(x), int(ii + contact_y))  # 상위 1/3 지점부터 그래프를 그리기 시작하므로 인덱스에 해당하는 ii 에 그만큼 추가해준다.
        if (x >= 0):
            k.append(q)
            left_lane_all_x.append(q[0])
            left_lane_all_y.append(q[1])
    for ii, x in enumerate(right_fitx):
        q = (int(x), int(ii + contact_y))
        if (x >= 0):
            l.append(q)
            right_lane_all_x.append(q[0])
            right_lane_all_y.append(q[1])
print('int left_lane_all_x[] = {', end='')
for i in range(0, len(left_lane_all_x)):
    print(left_lane_all_x[i], ',', end='')
    if i == len(left_lane_all_x) - 1:
        print(left_lane_all_x[i],'};')
        break
print(len(left_lane_all_x))
print(len(left_lane_all_y))
print(len(right_lane_all_x))
print(len(right_lane_all_x))

print('int left_lane_all_y[] = {', end='')
for i in range(0, len(left_lane_all_y)):
    print(left_lane_all_y[i], ',', end='')
    if i == len(left_lane_all_y) - 1:
        print(left_lane_all_y[i],'};')
        break

print('int right_lane_all_x[] = {', end='')

for i in range(0, len(right_lane_all_x)):
    print(right_lane_all_x[i], ',', end='')
    if i == len(right_lane_all_x) - 1:
        print(right_lane_all_x[i],'};')
        break

print('int right_lane_all_y[] = {', end='')
for i in range(0, len(right_lane_all_y)):
    print(right_lane_all_y[i], ',', end='')
    if i == len(right_lane_all_y) - 1:
        print(right_lane_all_y[i],'};')
        break


#print(f'left_lane_all_y = {left_lane_all_y}')

#print(f'right_lane_all_x = {right_lane_all_x}')
#print(f'right_lane_all_y = {right_lane_all_y[:]}')
k = np.array(k, np.int32)  # polylines 함수에 좌표는 np.array 형태를 유지해줘야하므로 추가한 코드이다.
# cv2.polylines(frame, [k], False, (235, 206, 135), thickness=7)  # 좌측 레인 : sky blue color
cv2.polylines(image_original, [k], False, (104, 255, 0), thickness=7)  # 좌측 레인 : fluorescent green color

l = np.array(l, np.int32)  # polylines 함수에 좌표는 np.array 형태를 유지해줘야하므로 추가한 코드이다.
# cv2.polylines(frame, [l], False, (160, 114, 0), thickness=7)  # 우측 레인 : deep blue color
cv2.polylines(image_original, [l], False, (19, 244, 239), thickness=7)  # 우측 레인 : fluorescent blue color

cv2.imshow("YOUNGIL", image_original)
cv2.imwrite("target.jpg", frame)

cv2.waitKey(0)
cv2.destroyAllWindows()

