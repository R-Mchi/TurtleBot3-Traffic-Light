# TurtleBot3-Traffic-Light
Turtlebot3 Traffic Light Detection is a system developed for the Turtlebot3 robot platform that allows it to detect and respond to traffic lights. The system utilizes the robot's camera to capture live images and performs color-based object detection to identify different traffic light signals, such as red, yellow, and green.
## Methodology
The system uses the OpenCV library for computer vision processing, where the captured images are converted to the HSV color space to enhance color detection. Specific color ranges are defined for each traffic light signal, such as red, yellow, and green, and masks are created to isolate these colors in the image.
Once the colors are detected, the system determines the current traffic light signal based on the predominant color in the image. If a red signal is detected, the robot stops its movement. If a yellow signal is detected, the robot moves at a slow speed. And if a green signal is detected, the robot moves at a regular speed.
![alt text](https://github.com/R-Mchi/TurtleBot3-Traffic-Light/blob/main/Screenshot from 2023-06-11 22-27-52.png?raw=true)
