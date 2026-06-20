#!/usr/bin/env python3
"""
ROS-нода навигации по ArUco-меткам.
Летит вперед по координатам aruco_map, публикует текущие координаты.
"""

import rospy
import math
from std_msgs.msg import String, Empty
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('aruco_navigator')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

coord_pub = rospy.Publisher('/navigator/coords', String, queue_size=10)
status_pub = rospy.Publisher('/mission/status', String, queue_size=10)

shutdown = False


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5,
                  frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if auto_arm:
        rospy.loginfo("[ARM] Takeoff")
    else:
        rospy.loginfo(f"Fly to ({x:.2f}, {y:.2f}, {z:.2f}) frame={frame_id}")
    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
        except rospy.ServiceException:
            continue
        if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
            break
        rospy.sleep(0.2)


def publish_coords():
    telem = get_telemetry(frame_id='aruco_map')
    msg = f"x={telem.x:.2f} y={telem.y:.2f} z={telem.z:.2f}"
    coord_pub.publish(data=msg)
    rospy.loginfo(msg)
    return telem.x, telem.y, telem.z


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    rospy.loginfo("[DISARM]")


def fly_forward(waypoints, altitude=1.5, speed=0.5):
    global shutdown
    navigate_wait(z=altitude, frame_id='body', auto_arm=True, speed=speed)
    shutdown = False

    for i, (x, y) in enumerate(waypoints):
        if shutdown:
            return
        navigate_wait(x=x, y=y, z=altitude, speed=speed, frame_id='aruco_map')
        publish_coords()
        status_pub.publish(data=f"WP {i+1}/{len(waypoints)}: ({x:.1f}, {y:.1f})")
        rospy.sleep(0.5)

    # Возврат
    navigate_wait(x=0, y=0, z=altitude, speed=speed, frame_id='aruco_map')
    navigate_wait(x=0, y=0, z=0.3, speed=0.3, frame_id='aruco_map', tolerance=0.15)
    land_wait()
    status_pub.publish(data="COMPLETED")


# Зигзаг по сетке ArUco 10x10
ZIGZAG = []
for row in range(10):
    xs = range(10) if row % 2 == 0 else range(9, -1, -1)
    for col in xs:
        ZIGZAG.append((col, row))


def mission_start_cb(msg):
    fly_forward(ZIGZAG)

def mission_land_cb(msg):
    global shutdown
    shutdown = True
    land_wait()

def mission_kill_cb(msg):
    global shutdown
    shutdown = True
    from clover import srv
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    if get_telemetry().armed:
        set_attitude(thrust=0)

rospy.Subscriber('/mission/start', Empty, mission_start_cb)
rospy.Subscriber('/mission/land', Empty, mission_land_cb)
rospy.Subscriber('/mission/kill', Empty, mission_kill_cb)

if __name__ == '__main__':
    fly_forward(ZIGZAG)
