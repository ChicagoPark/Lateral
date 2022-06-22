`Supported by the National Research Foundation of Korea(NRF) grant funded by the Korea government(MSIT)`

`Real-time Version of Lateral Distance Project:` https://github.com/ChicagoPark/Lateral_Realtime


## [1] Project Goal
<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/162565112-7c3be6ca-e65a-4fc4-8860-b762a577a78b.png">

`Goal: Obtaining accurate lateral distance between vehicles and adjacent lanes`


## [2] Project Motivation

> The `precise localization of nearby vehicles` in advanced driving assistance systems or autonomous
> driving systems is imperative for path planning or anomaly detection by monitoring and predicting
> the future behavior of the vehicles. Mainly, `the lateral position of a vehicle with respect to lane
> markers is essential information` for such tasks.


## [3] Essential Idea

> We `propose a novel method to accurately estimate the lateral distance` of a front vehicle to
> lane markers `by combining camera and LiDAR sensors` and the `fusion of deep neural networks` in
> 2D and 3D domains.


## [4]  Overall system architecture

<img width="750" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/162564892-2428b167-606e-46f7-bf85-bce40cf37345.jpeg">


### [4-1] (Detailed Process) Segmentation of a LiDAR point cloud

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671866-3e481ade-b01d-4497-ba55-78e4e0204abd.jpeg">

> We utilized the RANdom SAmple Consensus (RANSAC) function to `discriminate point clouds for road
> and obstacles` separately.


### [4-2] (Detailed Process) Projection of LiDAR points onto the image plane of a front-view camera

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671867-3a989e59-ce7f-42e8-a038-be97a671a85a.jpeg">

> We placed point clouds on the image from a 3D point cloud and assigned the color of points on the
> image according to the intensity of point cloud data for visualization. We `utilize projected output
> from the sensor dimension upgrading process for lane matching`.

### [4-3] (Detailed Process) Lane detection on Image using the Ultrafast-ResNet model

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671868-49052412-5aca-422d-b03f-2ed6ee0cb443.jpeg">

> We use `Ultrafase-ResNot model,` which can detect the lane in harsh weather conditions and various environments.
> Additionally, to get stable and constant lane detection results, we fitted each lane's points `using a quadratic
> curve` and `plotted the lane` as far as possible within a visible road area.


### [4-4] (Detailed Process) 3D object detection on 3D domain using the PV-RCNN model

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671870-02d2fc41-558f-4d3f-8ff0-4bd23c25bcfb.png">

> We use the `PV-RCNN model,` which considers the orientation of a vehicle and covers the vehicle with twelve lines
> to indicate bounding boxes. Through this 3D detection result, we `can estimate the lateral distance from the exact
> mid-low point of vehicles to the lanes`

### [4-5] (Detailed Process) Selection of LiDAR points on the lanes

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671872-1685554d-c865-43f7-901c-4b6de904084a.jpeg">

> To obtain matching point points on lanes, `first, we get the quadratic equations for the lane markers` on the image
> domain and get the corresponding pixels occupied by the curves. Next, we `search for all projected points on the
> pixels` and keep them in dynamic memory.
> Through this process, `we can upgrade the dimension reliably.`



### [4-6] (Detailed Process) Visualization of the resulting lateral distances from lanes to vehicles

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163671878-73a4c49c-0e20-4762-96e1-3c2ee0564301.png">

> We obtain the lateral distance by `calculating the minimum distance` from the mid-low point of the vehicle bounding
> box to the curve.
> Last but not least, we `represent the lateral distance` in meters using text markers, and we indicate the lateral
> locations of vehicles using pink line markers.


## [5] Expected Benefits

> Our framework provides the location of other vehicles regarding the lanes of the ego-vehicle, whereas previous
> work usually offers the vehicle location with respect to the ego-vehicle. We strongly believe that our work can
> provide more helpful information for detecting the anomaly behavior or the trajectory prediction of surrounding vehicles.
> Furthermore, the lateral distance values can be shared with networked vehicles around an ego-vehicle to strengthen traffic
> safety.
> Thus, our work can be a `valuable addition to vehicle communication and network research`.


## [extra] Dataset labeling

<img width="350" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/163672005-96184a6f-357b-4230-b6fb-21ff35ec6145.jpeg">

> As `there is no dataset for vehicle’s lateral distances` measured from the lanes of an ego-vehicle, we `created a dataset`
> by ourselves. We first extracted 1,281 frames suitable for this study from the KITTI dataset and then searched for the
> closest points to a vehicle on the left and right lanes of the ego vehicle.

----

<!--

## `Additional developments of this research would be updated in this repository soon. (June 2022)`

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
-->




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


## Abstract Paper and Poster (Nov 2021)
`Published full paper would be available in June, 2022.`

<img width="317" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/149541640-808f9392-58eb-4a47-841f-04c85de3b534.png"><img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/150671435-681e57cb-54b0-42df-8f66-32d344240a2d.png">

[Abstract_Paper.pdf](https://github.com/ChicagoPark/vehicle_lateral_positioning/files/7871472/Abstract_Paper.pdf)


## Certificate

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/149545018-cb9298d1-b643-4def-b39a-39a04bdfd9eb.png">


