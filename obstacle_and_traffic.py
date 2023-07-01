import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
import numpy as np
import subprocess

color_detection_flag = False


def set_camera_brightness(value):
    command = f"v4l2-ctl -d /dev/video0 -c brightness={value}"
    subprocess.call(command, shell=True)


def image_callback(msg):
    global color_detection_flag
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv_image1 = cv2.convertScaleAbs(cv_image, alpha=0.4, beta=0)

    # Proses deteksi lampu lalu lintas pada gambar
    hsv_image = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)

    # Mendeteksi warna merah
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])
    mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    pixel_count_red = cv2.countNonZero(mask_red)

    contours_red, _ = cv2.findContours(
        mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours_red:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(cv_image1, (x, y), (x + w, y + h), (0, 0, 255), 2)
        text = f"Red: {pixel_count_red}"
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        text_x = x
        text_y = y + h + text_size[1] + 5
        cv2.putText(cv_image1, text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Mendeteksi warna kuning
    lower_yellow1 = np.array([20, 100, 100])
    upper_yellow1 = np.array([39, 255, 255])
    mask_yellow1 = cv2.inRange(hsv_image, lower_yellow1, upper_yellow1)

    lower_yellow2 = np.array([20, 100, 100])
    upper_yellow2 = np.array([30, 255, 255])
    mask_yellow2 = cv2.inRange(hsv_image, lower_yellow2, upper_yellow2)

    mask_yellow = cv2.bitwise_or(mask_yellow1, mask_yellow2)
    pixel_count_yellow = cv2.countNonZero(mask_yellow)

    contours_yellow, _ = cv2.findContours(
        mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours_yellow:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(cv_image1, (x, y), (x + w, y + h), (0, 255, 255), 2)
        text = f"Yellow: {pixel_count_yellow}"
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        text_x = x
        text_y = y + h + text_size[1] + 5
        cv2.putText(cv_image1, text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # Mendeteksi warna hijau
    lower_green1 = np.array([40, 100, 100])
    upper_green1 = np.array([80, 255, 255])
    mask_green1 = cv2.inRange(hsv_image, lower_green1, upper_green1)

    lower_green2 = np.array([50, 100, 100])
    upper_green2 = np.array([70, 255, 255])
    mask_green2 = cv2.inRange(hsv_image, lower_green2, upper_green2)

    mask_green = cv2.bitwise_or(mask_green1, mask_green2)
    pixel_count_green = cv2.countNonZero(mask_green)

    contours_green, _ = cv2.findContours(
        mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours_green:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(cv_image1, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = f"Green: {pixel_count_green}"
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        text_x = x
        text_y = y + h + text_size[1] + 5
        cv2.putText(cv_image1, text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # cv2.putText(cv_image1, f"Red: {pixel_count_red}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    # cv2.putText(cv_image1, f"Yellow: {pixel_count_yellow}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    # cv2.putText(cv_image1, f"Green: {pixel_count_green}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Menampilkan hasil deteksi
    cv2.imshow("Ori", cv_image1)
    # cv2.imshow("Red", mask_red)
    # cv2.imshow("Yellow", mask_yellow)
    # cv2.imshow("Green", mask_green)
    cv2.waitKey(1)

    # Logika pergerakan robot berdasarkan deteksi warna lampu lalu lintas
    if cv2.countNonZero(mask_red) > 100:
        move.linear.x = 0
        move.angular.z = 0
        print("Lampu merah terdeteksi. Robot berhenti.")
        pub.publish(move)
        color_detection_flag = True

    elif cv2.countNonZero(mask_yellow) > 100:
        move.linear.x = 0.05
        move.angular.z = 0
        print("Lampu kuning terdeteksi. Robot bergerak dengan kecepatan 0.02.")
        pub.publish(move)
        color_detection_flag = True

    elif cv2.countNonZero(mask_green) > 100:
        move.linear.x = 0.05
        move.angular.z = 0
        print("Lampu hijau terdeteksi. Robot bergerak dengan kecepatan 0.05.")
        pub.publish(move)
        color_detection_flag = True
    else:
        color_detection_flag = False
        # move.linear.x = 0.05
        # move.angular.z = 0
        # pub.publish(move)


def laser_callback(msg):
    # print('depan [0] =', msg.ranges[0])
    # print('kiri [90] =', msg.ranges[90])
    # print('belakang [180] =', msg.ranges[180])

    if not color_detection_flag:
        if msg.ranges[0] > 0.25:
            move.linear.x = 0.05
            move.angular.z = 0
            print("Bergerak 0.05")

        # if msg.ranges[180] < 0.5:
        #     move.linear.x = 0.5
        #     move.angular.z = 0
        #     print("Bergerak 0.5")

        elif msg.ranges[0] < 0.25:
            move.linear.x = 0
            move.angular.z = 0
            print("Berhenti")

    # elif color_detection_flag == True:
    #     if msg.ranges[0] < 0.5:
    #         move.linear.x = 0
    #         move.angular.z = 0
    #         print("Macet, Berhenti")
    # else:
    #     move.linear.x = 0.05
    # Ubah ukuran dan tipe sesuai kebutuhan

    pub.publish(move)


rospy.init_node('turtlebot_control')

sub_image = rospy.Subscriber(
    "/usb_cam_node/image_raw", Image, image_callback)
sub_laser = rospy.Subscriber('/scan', LaserScan, laser_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()
set_camera_brightness(10)
rospy.spin()
