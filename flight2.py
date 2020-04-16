import cv2
from pyzbar import pyzbar
import numpy as np
from aruco_pose.msg import MarkerArray
import rospy
from clever import srv
from std_srvs.srv import Trigger
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clever.srv import SetLEDEffect
from mavros_msgs.srv import CommandBool

rospy.init_node('flight')

# Initialization
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)  # define proxy to ROS-service

bridge = CvBridge()

image_pub = rospy.Publisher('~image', Image, queue_size=1)

def image_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    color = color_recognition(cv_image)
    qr_data = qr_recognition(cv_image)

    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

base_height = 1.2
tolerance = 0.2

def max_mask(r, y, g):
    if r > y and r > g:
        return r, 'red'
    elif y > r and y > g:
        return y, 'yellow'
    elif g > r and g > y:
        return g, 'green'
    else:
        return 0, 'none'

def color_recognition(image):
    red = np.array([[0 , 70, 70], [8, 255, 255], [0, 255, 255]]) # low, high, true
    yellow = np.array([[20, 50, 50], [35, 255, 255], [30, 255, 255]])
    green = np.array([[40, 50, 50], [85, 255, 255], [60, 255, 255]])
    black = np.array([0, 0, 0])
    white = np.array([255, 255, 255])

    image = image[int(image.shape[0]*0.25) : int(image.shape[0]*0.75), int(image.shape[1]*0.3) : int(image.shape[1]*0.7), :] # image crop

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # hsv image

    red_mask = cv2.inRange(hsv_image, red[0], red[1])
    yellow_mask = cv2.inRange(hsv_image, yellow[0], yellow[1])
    green_mask = cv2.inRange(hsv_image, green[0], green[1])

    num, result = max_mask(np.count_nonzero(red_mask > 0), np.count_nonzero(yellow_mask > 0), np.count_nonzero(green_mask > 0)) # dominant color

    color = 'null'
    if result == 'red':
        hsv_image[red_mask > 0] = white
        hsv_image[red_mask <= 0] = black
        color = 'red'
        if num > 5:
            cv2.putText(image, color, (int(image.shape[1]/4), int(image.shape[0]/4)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)

    elif result == 'yellow':
        hsv_image[yellow_mask > 0] = white
        hsv_image[yellow_mask <= 0] = black
        color = 'yellow'
        if num > 5:
            cv2.putText(image, color, (int(image.shape[1]/4), int(image.shape[0]/4)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)

    elif result == 'green':
        hsv_image[green_mask > 0] = white
        hsv_image[green_mask <= 0] = black
        color = 'green'
        if num > 5:
            cv2.putText(image, color, (int(image.shape[1]/4), int(image.shape[0]/4)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 255), 2)

    return color

def qr_recognition(image):

    detectedBarcodes = pyzbar.decode(image)

    if len(detectedBarcodes) > 0:
        barcode = detectedBarcodes[0]
        cv2.putText(image, barcode.data.decode("utf-8"), (int(image.shape[1]/2), int(image.shape[0]/2)), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,255), 2)
        print("QR data:", barcode.data.decode("utf-8"))

        return barcode.data.decode("utf-8")

    else:
        cv2.putText(image, 'none', (int(image.shape[1]/4), int(image.shape[0]/4)), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,0), 2)

        return 'none'

def sort_flight_data(arr):
    size = len(arr)
    for i in range(size-1):
        for j in range(size-i-1):
            if arr[j][0] > arr[j + 1][0]:
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
    for i in range(size-1):
        for j in range(size-i-1):
            if arr[j][0] == arr[j+1][0] and arr[j][1] > arr[j+1][1]:
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;

def stat_print(t):
    print("Statistics after the first flight:")
    for i in t:
        if t[i][2] == 'red':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "have high temperature")
        if t[i][2] == 'yellow':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "have obscure state")
        if t[i][2] == 'green':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "healthy")

def stat_print_qr(t):
    print("Statistics after the second flight:")
    for i in t:
        if t[i][3] == 'COVID - 19':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "have COVID-19")
        if t[i][3] == 'Non COVID - 19':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "have just temperature, non COVID-19")
        if t[i][3] == 'healthy':
            print("Patient", i, "in x =", t[i][0], "y = ", t[i][1], "healthy")

def put_to_stat(targets, N, x, y, color):
    global sick_patients
    targets[N][2] = color
    if color == 'green':
        targets[N][3] = 'healthy'
    else:
        sick_patients.append([x, y, N])

def qr_put_to_stat(targets, N, x, y, qr_c):
    targets[N][3] = qr_c

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

def takeoff_wait(alt, speed=0.5, tolerance=tolerance):
    start = get_telemetry()
    navigate(z=alt, speed=0.5, frame_id='body', auto_arm=True)

    while not rospy.is_shutdown():
        if start.z + alt - get_telemetry().z < tolerance:
            break
        rospy.sleep(0.2)

    rospy.sleep(2)
    navigate(z=base_height, speed=0.5, frame_id='aruco_map', auto_arm=True)
    rospy.sleep(2)
    print("Takeoff over")

def navigate_wait(x, y, z, speed, frame_id, tolerance=tolerance):
    navigate(x=x, y=y, z=z, speed=0.7, frame_id=frame_id)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id=frame_id)
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)

    print("Reached point: ", x, y)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def detect(targets, patient, tolerance):

    navigate(z=-0.6, speed=0.5, frame_id='body')

    while not rospy.is_shutdown():
        if base_height - 0.6 - get_telemetry().z < tolerance:
            break
        rospy.sleep(0.2)

    set_position(frame_id='body')
    rospy.sleep(4)
    print("Ready to detect")

    c = color

    while c != 'red' or c != 'yellow' or c != 'green':
        c = color

    if c == 'red':
        print("OH NO, TEMPERATURE, CORONAVIRUS", c)
        set_effect(effect='blink', r=255, g=0, b=255)  # blink with red color
        rospy.sleep(2)
    elif c == 'yellow':
        print("YELLOW, WHAT?", c)
        set_effect(effect='blink', r=255, g=0, b=255)  # blink with yellow color
        rospy.sleep(2)
    elif c == 'green':
        print("U R FROM GREENPEACE?", c)
        rospy.sleep(2)

    put_to_stat(targets=targets, N=patient[2], x=patient[0], y=patient[1], color=c)
    return c

def qr_detect(targets, sick, tolerance):

    navigate(z=-0.6, speed=0.5, frame_id='body')

    while not rospy.is_shutdown():
        if base_height - 0.6 - get_telemetry().z < tolerance:
            break
        rospy.sleep(0.2)

    set_position(frame_id='body')
    rospy.sleep(5)
    print("Ready to detect")
    qr_code = qr_data

    if qr_code == 'COVID - 19':
        print("PRESS F, CORONAVIRUS")
    elif qr_code == 'Non COVID - 19':
        print("FUF, NO CORONA")
    elif qr_code == 'healthy':
        print("WE HAVE NEVER BEEN SO WRONG")

    qr_put_to_stat(targets=targets, N=sick[2], x=sick[0], y=sick[1], qr_c=qr_code)

    navigate(z=base_height-0.6, speed=0.5, frame_id='body')

    while not rospy.is_shutdown():
        if base_height - get_telemetry().z < tolerance:
            break
        rospy.sleep(0.2)

    set_position(x=sick[0], y=sick[1], z=base_height, frame_id='aruco_map')

    rospy.sleep(4)

def deliever(patient, col, tolerance=tolerance):

    if col == 'red' and col == 'yellow':

        alt = get_telemetry().z
        navigate(z=0.2-alt, speed=0.5, frame_id='body')

        while not rospy.is_shutdown():
            if 0.2 - get_telemetry().z < tolerance:
                break
            rospy.sleep(0.2)

        set_position(frame_id='body')

        rospy.sleep(3)
        print("Cargo delivered")

        navigate(z=base_height-0.2, speed=0.5, frame_id='body')

        while not rospy.is_shutdown():
            if base_height - get_telemetry().z < tolerance:
                break
            rospy.sleep(0.2)

        set_position(x=patient[0], y=patient[1], z=base_height, frame_id='aruco_map')

        rospy.sleep(2)

    else:
        navigate(z=base_height-0.2, speed=0.5, frame_id='body')

        while not rospy.is_shutdown():
            if base_height - get_telemetry().z < tolerance:
                break
            rospy.sleep(0.2)

        set_position(x=patient[0], y=patient[1], z=base_height, frame_id='aruco_map')

        print("Not need to deliever")
        rospy.sleep(3)


def detection_flight():

    global targets

    takeoff_wait(alt=base_height)  # fly to 1.2m height to detect markers

    home = get_telemetry(frame_id='aruco_map')

    print("Wait for 12 seconds")
    rospy.sleep(12)

    # check all positions
    for patient in patients:
        navigate_wait(x=patient[0], y=patient[1], z=base_height, frame_id='aruco_map', speed=1)
        rospy.sleep(5)
        c = detect(targets, patient, tolerance)
        rospy.sleep(5)
        deliever(patient, c)
        rospy.sleep(5)

    navigate_wait(x=home.x, y=home.y, z=base_height, speed=1, frame_id='aruco_map') # go home
    rospy.sleep(5)

    land_wait() # landing
    rospy.sleep(5)

    stat_print(targets)

def check_flight():

    global targets

    takeoff_wait(alt=base_height)  # fly to 1.2m height to detect markers

    home = get_telemetry(frame_id='aruco_map')

    print("Wait for 12 seconds")
    rospy.sleep(12)

    for sick in sick_patients:
        navigate_wait(x=sick[0], y=sick[1], z=base_height, frame_id='aruco_map', speed=1)
        rospy.sleep(5)
        qr_detect(targets, sick, tolerance)
        rospy.sleep(7)

    navigate_wait(x=home.x, y=home.y, z=base_height, speed=1, frame_id='aruco_map') # go home
    rospy.sleep(5)

    land_wait() # landing
    rospy.sleep(5)

    stat_print_qr(targets)


color = ''

patients = [[0, 2.72, 1],
            [0.72, 3.94, 2],
            [0.72, 1.5, 3],
            [2.88, 1.5, 4],
            [2.88, 3.94, 5],
            [2.16, 0.28, 6],
            [1.44, 2.72, 7],
            [1.44, 1.5, 8],
            [2.16, 2.72, 9],
            [3.6, 0.28, 10]]

sick_patients = []

targets = {
    1: [0, 2.72, '', ''],
    2: [0.72, 3.94, '', ''],
    3: [0.72, 1.5, '', ''],
    4: [2.88, 1.5, '', ''],
    5: [2.88, 3.94, '', ''],
    6: [2.16, 0.28, '', ''],
    7: [1.44, 2.72, '', ''],
    8: [1.44, 1.5, '', ''],
    9: [2.16, 2.72, '', ''],
    10: [3.6, 0.28, '', '']}

sort_flight_data(patients)

detection_flight()

arming(False)

print("Wait for 2 minutes")
rospy.sleep(120) # sleep for 2 minutes

arming(True)

check_flight()

arming(False)

print("Mission comlete!")

# THE END
