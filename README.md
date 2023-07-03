# TurtleBot3-Traffic-Light
Turtlebot3 Traffic Light Detection is a system developed for the Turtlebot3 robot platform that allows it to detect and respond to traffic lights. The system utilizes the robot's camera to capture live images and performs color-based object detection to identify different traffic light signals, such as red, yellow, and green.
## Setting Up
1. first, keep in mind that TurtleBot3 must be fully installed. You can follow the steps from this link
>https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/.
If at the time of launching the turtlebot autorace error then you can use the node that I have created, [usb_camera_publish.launch](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/blob/main/usb_camera_publish.launch) dan [turtlebot3_with_camera.launch](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/blob/main/turtlebot3_with_camera.launch) put both launch files in
>cd catkin_ws/src
2. After that, make sure you have Python installed on your laptop/computer. Python is used as a programming language to control robots, you can see the full code here
>[Obstacle and Traffic](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/edit/main/obstacle_and_traffic.py)
## Methodology
The system uses the OpenCV library for computer vision processing, where the captured images are converted to the HSV color space to enhance color detection. Specific color ranges are defined for each traffic light signal, such as red, yellow, and green, and masks are created to isolate these colors in the image.
Once the colors are detected, the system determines the current traffic light signal based on the predominant color in the image. If a red signal is detected, the robot stops its movement. If a yellow signal is detected, the robot moves at a slow speed. And if a green signal is detected, the robot moves at a regular speed.
![alt text](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/blob/main/mask.png?raw=true "Masking Colour")

The laser scanner continuously scans the area around the robot and provides distance measurements for objects in its field of view. These measurements are used to create a representation of the environment, known as a point cloud, which allows the robot to perceive the spatial layout of its surroundings. Using this point cloud data, the system applies obstacle detection algorithms to identify obstacles within the robot's vicinity. It analyzes the distance and spatial information to determine the presence and location of obstacles, such as walls, furniture, or other objects. Once an obstacle is detected, the system generates navigation commands to ensure the robot can navigate safely around it. The robot adjusts its movement by slowing down or coming to a complete stop depending on the proximity and size of the obstacle.
![alt text](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/blob/main/LDRscan.png?raw=true "Masking Colour")
