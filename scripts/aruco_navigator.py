#!/usr/bin/env python3
"""
ROS-нода навигации по ArUco-меткам.
Летит пошагово по сетке, начиная от текущей позиции.
Публикует координаты в /navigator/coords.
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
MAP_SIZE = 10


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.3,
                  frame_id='aruco_map', auto_arm=False, tolerance=0.3):
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


def get_aruco_pos():
    try:
        telem = get_telemetry(frame_id='aruco_map')
        return telem.x, telem.y, telem.z
    except rospy.ServiceException:
        return None


def publish_coords():
    pos = get_aruco_pos()
    if pos:
        msg = f"x={pos[0]:.2f} y={pos[1]:.2f} z={pos[2]:.2f}"
        coord_pub.publish(data=msg)
        rospy.loginfo(msg)
    return pos


def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    rospy.loginfo("[DISARM]")


def nearest_grid(x, y):
    return max(0, min(MAP_SIZE - 1, round(x))), max(0, min(MAP_SIZE - 1, round(y)))


def fly_step(gx, gy, altitude, speed):
    navigate_wait(x=gx, y=gy, z=altitude, speed=speed, frame_id='aruco_map', tolerance=0.3)
    pos = publish_coords()
    if pos and (abs(pos[0]) > MAP_SIZE + 2 or abs(pos[1]) > MAP_SIZE + 2):
        rospy.logwarn(f"OUT OF MAP bounds: ({pos[0]:.2f}, {pos[1]:.2f})")
        return False
    return True


def fly_forward(altitude=1.2, speed=0.3):
    global shutdown

    navigate_wait(z=altitude, frame_id='body', auto_arm=True, speed=speed)

    # Ждём aruco_map
    rospy.loginfo("Waiting for aruco_map...")
    start_pos = None
    for _ in range(150):
        if shutdown:
            land_wait()
            return
        pos = get_aruco_pos()
        if pos and abs(pos[0]) < 20 and abs(pos[1]) < 20:
            start_pos = pos
            rospy.loginfo(f"aruco_map ready: ({pos[0]:.2f}, {pos[1]:.2f})")
            break
        rospy.sleep(0.2)

    if start_pos is None:
        rospy.logwarn("aruco_map unavailable — landing")
        land_wait()
        return

    shutdown = False

    sx, sy = nearest_grid(start_pos[0], start_pos[1])
    rospy.loginfo(f"Start grid: ({sx}, {sy})")

    if not fly_step(sx, sy, altitude, speed):
        land_wait()
        return
    rospy.sleep(1)

    # Зигзаг от текущей строки
    if sy >= MAP_SIZE - 2:
        rows = range(MAP_SIZE - 1, -1, -1)
    else:
        rows = range(MAP_SIZE)

    for row_y in rows:
        if shutdown:
            return
        cols = range(MAP_SIZE) if row_y % 2 == 0 else range(MAP_SIZE - 1, -1, -1)
        for col_x in cols:
            if shutdown:
                return
            if row_y == rows[0]:
                if sy >= MAP_SIZE - 2 and col_x < sx:
                    continue
                if sy < MAP_SIZE - 2 and col_x > sx:
                    continue

            status_pub.publish(data=f"WP ({col_x}, {row_y})")
            if not fly_step(col_x, row_y, altitude, speed):
                rospy.logwarn("Aborting — out of map")
                land_wait()
                return
            rospy.sleep(0.3)

    status_pub.publish(data="Returning home")
    navigate_wait(x=0, y=0, z=altitude, speed=speed, frame_id='aruco_map', tolerance=0.3)
    navigate_wait(x=0, y=0, z=0.3, speed=0.2, frame_id='aruco_map', tolerance=0.15)
    land_wait()
    status_pub.publish(data="COMPLETED")


def mission_start_cb(msg):
    fly_forward()

def mission_land_cb(msg):
    global shutdown
    shutdown = True
    land_wait()

def mission_kill_cb(msg):
    global shutdown
    shutdown = True
    set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
    if get_telemetry().armed:
        set_attitude(thrust=0)

rospy.Subscriber('/mission/start', Empty, mission_start_cb)
rospy.Subscriber('/mission/land', Empty, mission_land_cb)
rospy.Subscriber('/mission/kill', Empty, mission_kill_cb)

if __name__ == '__main__':
    fly_forward()
