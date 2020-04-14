import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clever.srv import SetLEDEffect

def viewImage(image):
    cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
    cv2.imshow('Display', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def max_mask(r, y, g):
    if r > y and r > g:
        return 'red'
    elif y > r and y > g:
        return 'yellow'
    else:
        return 'green'

def color_recognition(image):
    red_low = np.array([0 , 70, 70])
    red_high = np.array([10, 255, 255])
    red = np.array([0, 255, 255])

    yellow_low = np.array([25 , 70, 70])
    yellow_high = np.array([35, 255, 255])
    yellow = np.array([30, 255, 255])

    green_low = np.array([50 , 70, 70])
    green_high = np.array([70, 255, 255])
    green = np.array([60, 255, 255])

    black = np.array([0, 0, 0])

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_mask = cv2.inRange(hsv_image, red_low, red_high)
    yellow_mask = cv2.inRange(hsv_image, yellow_low, yellow_high)
    green_mask = cv2.inRange(hsv_image, green_low, green_high)

    result = max_mask(np.count_nonzero(red_mask > 0), np.count_nonzero(yellow_mask > 0), np.count_nonzero(green_mask > 0))

    set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

    if result == 'red':
        print("OH MY YOU CAN SEE SOME BLOOD")
        hsv_image[red_mask > 0] = red
        hsv_image[red_mask <= 0] = black
        color_num = 1
        set_effect(effect='blink', r=255, g=0, b=0)  # blink with red color
        rospy.sleep(2)

    elif result == 'yellow':
        print("SUN UNDER YOUR FEET")
        hsv_image[yellow_mask > 0] = yellow
        hsv_image[yellow_mask <= 0] = black
        color_num = 2
        set_effect(effect='blink', r=255, g=255, b=0)  # blink with yellow color
        rospy.sleep(2)

    elif result == 'green':
        print("DO U WANT SOME GREEN?")
        hsv_image[green_mask > 0] = green
        hsv_image[green_mask <= 0] = black
        color_num = 3
        set_effect(effect='blink', r=0, g=255, b=0)  # blink with green color
        rospy.sleep(2)

    rgb_again = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(rgb_again, cv2.COLOR_RGB2GRAY)
    ret, threshold = cv2.threshold(gray, 90, 255, 0)
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, contours, -1, (255, 0, 0), 2)

    return color_num

# image = cv2.imread('pictures/Green.jpg')

def give_me_color_from_camera():
    rospy.init_node('color_detection')

    bridge = CvBridge()

    def image_callback(data):
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        color_num = color_recognition(cv_image)

    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

    rospy.sleep(2)

    return color_num
