#!/usr/bin/env python3
"""
ROS-нода автономной инспекции солнечных панелей.
Определяет: координаты, перегрев (цвет площадки), загрязнения, LED-индикация.
Публикует в /solar (Image с обведёнными контурами) и /buildings (String с отчётом).
"""

import rospy
import cv2
import math
import json
import os
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Empty, ColorRGBA
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import tf2_ros
import tf2_geometry_msgs

# image_geometry может не быть установлен
try:
    import image_geometry
    HAS_IMAGE_GEOMETRY = True
except ImportError:
    HAS_IMAGE_GEOMETRY = False

rospy.init_node('solar_inspector', disable_signals=True)

# Сервисы
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
land = rospy.ServiceProxy('land', Trigger)
set_led = rospy.ServiceProxy('set_led', srv.SetLEDEffect)

# Публикаторы
solar_pub = rospy.Publisher('/solar', Image, queue_size=10)
mask_pub = rospy.Publisher('/solar/mask', Image, queue_size=10)
report_pub = rospy.Publisher('/buildings', String, queue_size=10)
status_pub = rospy.Publisher('/mission/status', String, queue_size=10)

# TF
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Камера — пробуем image_geometry, fallback на упрощённую проекцию
camera_model = None
if HAS_IMAGE_GEOMETRY:
    try:
        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(rospy.wait_for_message('main_camera/camera_info', CameraInfo))
    except Exception:
        camera_model = None

bridge = CvBridge()

# Цвета HSV для определения перегрева
HEAT_HSV = {
    'yellow':  ((20, 100, 100), (35, 255, 255)),   # желтая = нормальное состояние
    'orange':  ((10, 100, 100), (20, 255, 255)),    # оранжевая = некритический перегрев
    'red_low': ((0, 100, 100), (10, 255, 255)),     # красная (низкий диапазон)
    'red_high':((170, 100, 100), (180, 255, 255)),  # красная (верхний диапазон H)
}

# Расширенный диапазон для зелёного (загрязнения)
CONTAMINATION_HSV = ((30, 30, 30), (90, 255, 255))

HEAT_LABELS = {
    'yellow': 'нормальное состояние',
    'orange': 'некритический перегрев',
    'red': 'срочный ремонт',
}

HEAT_LED_COLORS = {
    'yellow': (255, 200, 0),
    'orange': (255, 140, 0),
    'red': (255, 0, 0),
}

# Данные о панелях
panels_data = []
report_lines = []
inspected_panels = []
shutdown_initiated = True


def load_panels_data():
    """Загрузка данных о панелях из JSON, сгенерированного gen_solar_farm.py."""
    global panels_data
    search_paths = [
        '/home/clover/nto_project/panels_data.json',
        'panels_data.json',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'panels_data.json'),
    ]
    for json_path in search_paths:
        try:
            with open(json_path, 'r') as f:
                panels_data = json.load(f)
            rospy.loginfo(f"Loaded {len(panels_data)} panels from {json_path}")
            return
        except FileNotFoundError:
            continue
    rospy.logwarn("panels_data.json not found, will rely on camera detection only")
    panels_data = []


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5,
                  frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    """Навигация дрона в указанную точку и ожидание достижения цели."""
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    if auto_arm:
        rospy.loginfo("[ARM] Takeoff")
    else:
        rospy.loginfo(f"Flight to {frame_id}: ({x:.2f}, {y:.2f}, {z:.2f})")

    while not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id='navigate_target')
        except rospy.ServiceException:
            continue
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def land_wait():
    """Ожидание посадки и disarm."""
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)
    rospy.loginfo("[DISARM]")


def is_near_inspected(x, y, threshold=1.0):
    """Проверяет, была ли уже обнаружена панель рядом с текущими координатами."""
    for panel in inspected_panels:
        dist = math.sqrt((panel['x'] - x) ** 2 + (panel['y'] - y) ** 2)
        if dist < threshold:
            return True
    return False


def detect_heat_color(hsv_img):
    """Определение цвета перегрева по индикационной площадке."""
    for color_name, (low, high) in HEAT_HSV.items():
        mask = cv2.inRange(hsv_img, np.array(low), np.array(high))
        if cv2.countNonZero(mask) > 50:
            if color_name == 'red_low' or color_name == 'red_high':
                return 'red'
            return color_name
    return None


def detect_contamination(hsv_img):
    """Подсчёт и обведение контуров объектов загрязнения (зелёные площадки)."""
    low, high = CONTAMINATION_HSV
    mask = cv2.inRange(hsv_img, np.array(low), np.array(high))

    # Морфологическая очистка: сначала закрываем дыры, потом убираем шум
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Фильтрация совсем маленьких шумов (понижен порог)
    filtered = [c for c in contours if cv2.contourArea(c) > 10]

    return len(filtered), filtered, mask


def set_led_color(color_name, duration=5):
    """Установка LED-индикации в цвет перегрева на duration секунд."""
    r, g, b = HEAT_LED_COLORS.get(color_name, (255, 255, 255))
    try:
        set_led(effect='fill', r=r, g=g, b=b)
        rospy.loginfo(f"LED set to {color_name} for {duration}s")
        rospy.sleep(duration)
        set_led(effect='off')
    except rospy.ServiceException:
        rospy.logwarn("LED service not available")


def inspect_panel(panel_idx, panel_x, panel_y):
    """Инспекция одной солнечной панели."""
    global shutdown_initiated, report_lines, inspected_panels

    if shutdown_initiated:
        return

    # Защита от повторной инспекции
    if is_near_inspected(panel_x, panel_y, threshold=0.8):
        rospy.loginfo(f"Panel at ({panel_x:.2f}, {panel_y:.2f}) already inspected. Skipping.")
        return

    rospy.loginfo(f"Inspecting panel #{panel_idx + 1} at ({panel_x:.2f}, {panel_y:.2f})")
    navigate_wait(x=panel_x, y=panel_y, z=1.5, speed=0.3, frame_id='aruco_map')
    rospy.sleep(1)

    img_msg = rospy.wait_for_message('/main_camera/image_raw', Image, timeout=5)
    img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    heat_color = detect_heat_color(hsv)
    if heat_color:
        heat_label = HEAT_LABELS[heat_color]
        rospy.loginfo(f"  Heat state: {heat_label} ({heat_color})")
        set_led_color(heat_color, duration=5)
    else:
        heat_label = "не определено"
        rospy.logwarn("  Heat color not detected")

    cont_count, cont_contours, cont_mask = detect_contamination(hsv)
    rospy.loginfo(f"  Contamination objects: {cont_count}")

    # Публикация маски для отладки
    mask_bgr = cv2.cvtColor(cont_mask, cv2.COLOR_GRAY2BGR)
    mask_msg = bridge.cv2_to_imgmsg(mask_bgr, encoding='bgr8')
    mask_pub.publish(mask_msg)

    # Обведение контуров на изображении
    annotated = img.copy()

    if cont_count > 0:
        # Красные контуры загрязнений
        cv2.drawContours(annotated, cont_contours, -1, (0, 0, 255), 3)

        # Жёлтые прямоугольники с подписями
        for i, contour in enumerate(cont_contours):
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(annotated, f"Cont {i+1}", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Общий текст
    cv2.putText(annotated, f"Panel #{panel_idx + 1}: {heat_label}, {cont_count} cont.",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    solar_msg = bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
    solar_pub.publish(solar_msg)

    telem = get_telemetry(frame_id='aruco_map')
    panel_coord_x = telem.x
    panel_coord_y = telem.y

    line = f"Солнечная панель №{panel_idx + 1}: {panel_coord_x:.1f} {panel_coord_y:.1f}, {heat_label}, {cont_count}"
    report_lines.append(line)
    rospy.loginfo(f"  Report: {line}")

    inspected_panels.append({
        'index': panel_idx + 1,
        'x': panel_coord_x,
        'y': panel_coord_y,
        'heat': heat_label,
        'contamination': cont_count,
    })


def main():
    """Основная миссия инспекции."""
    global shutdown_initiated, report_lines

    load_panels_data()

    # Взлёт
    navigate_wait(z=1.5, frame_id='body', auto_arm=True, speed=0.5)
    shutdown_initiated = False

    if panels_data:
        rospy.logwarn("panels_data is empty. Searching via zigzag pattern.")
        waypoints = [
            (1, 1.5), (3, 1.5), (5, 1.5), (7, 1.5), (9, 1.5),
            (9, 3.5), (7, 3.5), (5, 3.5), (3, 3.5), (1, 3.5),
            (1, 5.5), (3, 5.5), (5, 5.5), (7, 5.5), (9, 5.5),
            (9, 7.5), (7, 7.5), (5, 7.5), (3, 7.5), (1, 7.5),
            (1, 9.5), (3, 9.5), (5, 9.5), (7, 9.5), (9, 9.5),
        ]
        for x, y in waypoints:
            if shutdown_initiated:
                return

            if is_near_inspected(x, y, threshold=1.5):
                continue

            navigate_wait(x=x, y=y, z=1.5, speed=0.3, frame_id='aruco_map')
            rospy.sleep(1)

            img_msg = rospy.wait_for_message('/main_camera/image_raw', Image, timeout=5)
            img = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            heat_color = detect_heat_color(hsv)
            cont_mask_sum = cv2.inRange(hsv, np.array(CONTAMINATION_HSV[0]),
                                        np.array(CONTAMINATION_HSV[1])).sum()

            if heat_color or cont_mask_sum > 50:
                telem = get_telemetry(frame_id='aruco_map')
                if not is_near_inspected(telem.x, telem.y, threshold=1.0):
                    inspect_panel(len(inspected_panels), telem.x, telem.y)

    # Возвращение на старт
    rospy.loginfo("Returning to start")
    navigate_wait(x=0, y=0, z=1.5, speed=0.5, frame_id='aruco_map')
    navigate_wait(x=0, y=0, z=0.3, speed=0.3, frame_id='aruco_map', tolerance=0.15)
    land_wait()

    full_report = '\n'.join(report_lines)
    report_pub.publish(data=full_report)
    status_pub.publish(data="COMPLETED")

    with open('report.txt', 'w') as f:
        f.write(full_report + '\n')
    rospy.loginfo(f"Report saved: {full_report}")

    shutdown_initiated = True


# Обработка команд от веб-интерфейса
def mission_start_cb(msg):
    global shutdown_initiated
    shutdown_initiated = False
    main()


def mission_land_cb(msg):
    global shutdown_initiated
    shutdown_initiated = True
    land_wait()


def mission_kill_cb(msg):
    global shutdown_initiated
    shutdown_initiated = True
    if get_telemetry().armed:
        set_attitude(thrust=0)


rospy.Subscriber('/mission/start', Empty, mission_start_cb)
rospy.Subscriber('/mission/land', Empty, mission_land_cb)
rospy.Subscriber('/mission/kill', Empty, mission_kill_cb)


if __name__ == '__main__':
    import sys
    if '--land' not in sys.argv and '--kill' not in sys.argv:
        main()
        rospy.spin()
    elif '--land' in sys.argv:
        land_wait()
        exit()
    elif '--kill' in sys.argv:
        if get_telemetry().armed:
            set_attitude(thrust=0)
        exit()