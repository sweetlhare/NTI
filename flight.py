#coding=utf-8
import rospy
from aruco_pose.msg import MarkerArray
import rospy
from clever import srv
from std_srvs.srv import Trigger
import mycolor
import mystat

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

# [x, y, N]
patients = [[0, 2, 1],
            [1, 1, 3],
            [1, 3, 2],
            [2, 2, 7],
            [2, 1, 8],
            [3, 0, 6],
            [3, 2, 9],
            [4, 1, 4],
            [4, 3, 5],
            [5, 0, 10]]

def takeoff_wait(alt, speed=0.5, tolerance=0.1):
    start = get_telemetry()
    navigate(z=alt, speed=speed, frame_id='body', auto_arm=True)

    while not rospy.is_shutdown():
        if start.z + alt - get_telemetry().z < tolerance:
            break
        rospy.sleep(0.2)

    print('Takeoff over')

def navigate_wait(x, y, z, speed, frame_id, tolerance=0.1):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id=frame_id)
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)

    print('Reached point: ({0.3d}, {1.3d})', x, y)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def detection_flight():

    takeoff_wait(alt=0.7)  # fly to 0.7m height to detect markers

    home = get_telemetry(frame_id='aruco_map')

    # check all positions
    for patient in patients:
        navigate_wait(x=patient[0], y=patient[1], z=0.7, frame_id='aruco_map', speed=1)
        color_num = mycolor.give_me_color_from_camera() # give color
        mystat.put_to_stat(N=patient[2], x=patient[0], y=patient[1], color_num) # put color to stat
        # ЗДЕСЬ НУЖЕН МОДУЛЬ ДЛЯ ВЫГРУЗКИ ТЕСТА

    navigate_wait(x=home.x, y=home.y, z=0.7, speed=1, frame_id='aruco_map') # go home

    land_wait() # landing

    rospy.sleep(120) # sleep for 2 minutes

# deliever test for COVID
def check_flight():

    data = mystat.give_me_COVID_data() #dict with targets {N:[x, y, num_color]}

    takeoff_wait(alt=0.7)  # fly to 0.7m height to detect markers

    home = get_telemetry(frame_id='aruco_map')

    # check all positions
    for N in data:
        navigate_wait(x=data[N][0], y=data[N][0], z=0.7, frame_id='aruco_map', speed=1)
        # НАПИСАТЬ МОДУЛЬ QR
        # НАПИСАТЬ МОДУЛЬ ДЛЯ ДОБАВЛЯЕНИЯ СТАТИСТИКИ ПО QR

    navigate_wait(x=home.x, y=home.y, z=0.7, speed=1, frame_id='aruco_map') # go home

    land_wait() # landing
