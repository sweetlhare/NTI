#coding=utf-8
import rospy
from aruco_pose.msg import MarkerArray
import rospy
from clever import srv
from std_srvs.srv import Trigger
import mycolor

rospy.init_node('test_flight')

# ... Инициализация сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)



def markers_callback(msg):
    print 'Detected markers:'
    for marker in msg.markers:
        print 'Marker: %s' % marker

# Подписываемся. При получении сообщения в топик aruco_detect/markers будет вызвана функция markers_callback.
rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)

rospy.spin()

# Вначале необходимо взлететь, чтобы коптер увидел карту меток и появился фрейм aruco_map:
# Карта 4*4
# облет угловых меток
navigate(x=0, y=0, z=0.7, speed=1, frame_id='aruco_map')
time.sleep(5)
navigate(x=4, y=0, z=0.7, speed=1, frame_id='aruco_map')
time.sleep(5)
navigate(x=4, y=4, z=0.7, speed=1, frame_id='aruco_map')
time.sleep(5)
navigate(x=0, y=4, z=0.7, speed=1, frame_id='aruco_map')
time.sleep(5)
navigate(x=0, y=0, z=0.7, speed=1, frame_id='aruco_map')
time.sleep(5)

# Полет в координату 2:2 маркерного поля, высота 2 метра
  # полет в координату 2:2, высота 3 метра
