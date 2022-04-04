# vehicle_lateral_positioning
`Lateral distance, Lateral position, Sensor fusion, Deep learning, Neural network, Autonomous vehicle`

`Real-time Lateral:` https://github.com/ChicagoPark/Lateral_Realtime

<img width="317" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/149541640-808f9392-58eb-4a47-841f-04c85de3b534.png"><img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/150671435-681e57cb-54b0-42df-8f66-32d344240a2d.png">

[Abstract_Paper.pdf](https://github.com/ChicagoPark/vehicle_lateral_positioning/files/7871472/Abstract_Paper.pdf)


## [1] Project Goal
<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/139383767-c6116f15-713e-4ddb-9500-605f346a84ea.jpeg">

`Goal: Obtaining accurate lateral distance between vehicles and adjacent lanes`


## [2] Project Motivation
`In advanced driving assistance systems or autonomous driving systems, accurate localization of surrounding vehicles is very important for path planning and anomaly detection through monitoring and predicting the movement and behavior of the vehicles.`

## [3] Essential Idea
`We propose a novel method to accurately estimate the lateral distance of a nearby vehicle to lane markers by the fusion of vision and lidar sensors as well as the fusion of deep neural networks in 2D and 3D space.`

## [4] Project Pipeline

<img width="600" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/139428433-30e16219-0120-427c-8734-0794f9f40f71.png">

<img width="600" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/151820935-0ea2ff9a-f51b-4e29-bf96-2df0137be4ef.png">

## [5] Result

<img width="600" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/151821185-27cdca71-a7a0-4550-abd4-c7e4576170c8.png">

## [6] Expected Benefits
<img width="600" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/151821453-0446fe68-bac1-4a4c-b7a4-f920bce0f7bd.png">


## [7] Dataset preparation
`Using KITTI Dataset to label 3D point on lanes.`



##  CODE Explanation

### 1️⃣1_Lane_2D.py
```python
Detect the lanes on the image. (output: Pixel locations of lanes)
```
### 2️⃣2_Matching.py
```python
Match the corresponding point cloud with lane pixels (output: Coordinates of matched 3D points)
```
### 3️⃣3_Lane_Equation.py
```python
Get quadratic equations on 3D (output: coefficient of quadratic equations)
```
### 4️⃣4_Marking.py
```python
Visualize overlapped lanes (output: visualizing RVIZ)
```

### etc: label_plotting.py
```python
Visualize label data
```


-------
## Paperwork Ideabank
1. save lane detection output into the text file for csv file
2. get the '1.' value through method
-------

-------
## Roles
Bounding Box Visualization: /home/kaai/chicago_ws/src/kitti_visualizer/launch/object_visualizer.launch
Label Points Visualization: /home/kaai/chicago_ws/src/first_pkg/src/label_plotting.py


-------
<!--
## Minimum Distance Code
```bash
[reference]
[1] https://stackoverflow.com/questions/19101864/find-minimum-distance-from-point-to-complicated-curve
[2] https://shapely.readthedocs.io/en/stable/manual.html#points
```
```python
import numpy as np
import shapely.geometry as geom
from shapely.geometry import Point
import matplotlib.pyplot as plt

class NearestPoint(object):
    def __init__(self, line, ax):
        self.line = line
        self.ax = ax
        ax.figure.canvas.mpl_connect('button_press_event', self)

    def __call__(self, x_value, y_value):
        #x, y = event.xdata, event.ydata
        x, y = x_value, y_value
        point = geom.Point(x, y)
        distance = self.line.distance(point)
        self.draw_segment(point)
        print('Distance to line:', distance)

    def draw_segment(self, point):
        point_on_line = line.interpolate(line.project(point))
        self.ax.plot([point.x, point_on_line.x], [point.y, point_on_line.y], 
                     color='red', marker='o', scalex=False, scaley=False)
        fig.canvas.draw()

if __name__ == '__main__':
    coords = np.loadtxt('left_points.txt')
    r_coords = np.loadtxt('right_points.txt')
    
    # define object locations
    object_list = [[5.5, 10.2],[-16,3]]
    point_list = []
    
    # get the Point objects corresponding to vehicle
    for i in object_list:
        point_list.append(Point(i[0], i[1]))
        
    line = geom.LineString(coords)
   
    # Plotting Section
    fig, ax = plt.subplots()
    ax.plot(*coords.T)
    ax.plot(*r_coords.T)
    ax.axis('equal')
    # Set the frame
    ax.set_xlim(-30, 30)
    ax.set_ylim(-30, 30)
    
    distance_class = NearestPoint(line, ax)
    
    for i in range(len(point_list)):
        distance_class.draw_segment(point_list[i])
    
    
    plt.show()
```
-->



<!--
Code Movement
1️⃣  이미지 상 Lane Detection , 점 추출

~/lee_ws/src/AB3DMOT $ python KaAI_Image_Process.py

출력하는 것 : 레인에 대한 픽셀

2️⃣  Projection 및 포인트 매칭작업

작업공간

/home/kaai/chicago_ws/src/first_pkg/src/vehicle_lateral_positioning

~/chicago_ws $ rosrun first_pkg KaAI_KITTI_Point_Processing

출력하는 것 : Lane 에 대해 매칭하여 얻은 점

3️⃣  3D 상에 찍힌 점에 대해 다시 방정식을 도출하기

3D Point Equation

~/lee_ws/src/AB3DMOT$ python 3D_Point_Equation.py

4️⃣  3D 상에 해당좌표에 대해 Marker 를 명시해주기

rosrun first_pkg
pcd_with_intensity --m=bin2pcd --b=/home/kaai/chicago_ws/src/first_pkg/src/KITTI/bin/
--p=/home/kaai/chicago_ws/src/first_pkg/src/KITTI/pcd/






### [3-1]: Projection based Point Cloud on image

https://github.com/ChicagoPark/LiDAR_Projection

### [3-2] : Lane Detection on Image

#### Getting Quadratic Equation about Lane on image domain


### [3-3] : Finding points on 3D corresponding to Lane Detection on Image


#### `Obtain quadratic function for the lane on the 3D, and the overlapping part of the Point cloud is expressed with red columns.`


### [3-4]: A quadratic curve is obtained using the points on 3D.
'''

#### Quadratic Function about Lane on 3D domain

### [3-5]: Measure the shortest distance to the target point and lane of the object detection.


##  코드 움직임
1️⃣  이미지 상 Lane Detection , 점 추출

~/lee_ws/src/AB3DMOT $ python KaAI_Image_Process.py

출력하는 것 : 레인에 대한 픽셀

2️⃣  Projection 및 포인트 매칭작업

~/chicago_ws $ rosrun first_pkg KaAI_Point_Processing

출력하는 것 : Lane 에 대해 매칭하여 얻은 점

3️⃣  3D 상에 찍힌 점에 대해 다시 방정식을 도출하기

~/lee_ws/src/AB3DMOT $ python 3D_Point_Equation.py

4️⃣  3D 상에 해당좌표에 대해 Marker 를 명시해주기

~/chicago_ws $ rosrun first_pkg 3D_Lane_Marker

-->

## Certificate

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/149545018-cb9298d1-b643-4def-b39a-39a04bdfd9eb.png">


