# vehicle_lateral_positioning
`Lateral distance, Lateral position, Sensor fusion, Deep learning, Neural network, Autonomous vehicle`

<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/140629396-1adf36df-49ab-406e-abe4-805d8828d99e.jpg">

[vehicle_lateral_positioning.pdf](https://github.com/ChicagoPark/vehicle_lateral_positioning/files/7491556/vehicle_lateral_positioning.pdf)

## [1] Project Goal
<img width="300" alt="Distance Picture" src="https://user-images.githubusercontent.com/73331241/139383767-c6116f15-713e-4ddb-9500-605f346a84ea.jpeg">

`주변 차량의 차선에 대한 정확한 횡방향 거리 정보를 얻어오는 것이 목표`

## [2] Project Motivation
`In advanced driving assistance systems or autonomous driving systems, accurate localization of surrounding vehicles is very important for path planning and anomaly detection through monitoring and predicting the movement and behavior of the vehicles.`

## [3] Essential Idea
`We propose a novel method to accurately estimate the lateral distance of a nearby vehicle to lane markers by the fusion of vision and lidar sensors as well as the fusion of deep neural networks in 2D and 3D space.`

## [3] Project Pipeline

<img width="800" alt="Overall_Pipeline" src="https://user-images.githubusercontent.com/73331241/139428433-30e16219-0120-427c-8734-0794f9f40f71.png">


### [3-1] : 라이다의 이미지 상 Projection - (1) 전체에 대한 것, (2) 레인에 대한 것



### [3-2] : Image 상의 Lane Detection


#### Quadratic Function about Lane on image domain


### [3-3] : Image 상의 Lane Detction 에 대응되는 3D 상의 Point 찾기


#### `Obtain quadratic function for the lane on the 3D, and the overlapping part of the Point cloud is expressed with red columns.`


### [3-4] : 3D 상의 Point 를 활용하여, 2차 곡선을 얻는다.



#### Quadratic Function about Lane on 3D domain

### [3-5] : Object Detection 의 타겟지점과 Lane 까지의 최단거리를 측정


## 코드 움직임
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

