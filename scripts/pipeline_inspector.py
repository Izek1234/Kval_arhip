#!/usr/bin/env python3
"""
Нода автономного полёта дрона вдоль нефтяного трубопровода.
Обнаруживает врезки по изображению с камеры и публикует их координаты.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String, Empty, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tf.transformations as tft

try:
    from clover import srv
    from clover.srv import Navigate, NavigateGlobal, SetPosition, GetTelemetry, SetAttitude
except ImportError:
    # Если clover не доступен (для тестирования вне симуляции)
    srv = None


class PipelineInspector:
    """Автономный инспектор трубопровода."""

    def __init__(self):
        rospy.init_node('pipeline_inspector', log_level=rospy.INFO)

        self.bridge = CvBridge()
        self.detected_branches = []
        self.current_pose = None
        self.mission_active = False
        self.pipe_segments = []  # сегменты трубы из world-файла
        self.flight_height = 1.5  # высота полёта над трубой (м)
        self.search_radius = 0.3  # радиус поиска врезок (м)
        self.min_branch_dist = 0.5  # минимальное расстояние между врезками для фильтрации дублей

        # Сервисы Клевера
        self.navigate_srv = None
        self.navigate_global_srv = None
        self.get_telem_srv = None
        self.set_position_srv = None
        if srv:
            try:
                rospy.wait_for_service('navigate', timeout=5)
                self.navigate_srv = rospy.ServiceProxy('navigate', Navigate)
                rospy.wait_for_service('navigate_global', timeout=5)
                self.navigate_global_srv = rospy.ServiceProxy('navigate_global', NavigateGlobal)
                rospy.wait_for_service('get_telemetry', timeout=5)
                self.get_telem_srv = rospy.ServiceProxy('get_telemetry', GetTelemetry)
                rospy.wait_for_service('set_position', timeout=5)
                self.set_position_srv = rospy.ServiceProxy('set_position', SetPosition)
            except rospy.ROSException:
                rospy.logwarn("Clover services not available, running in simulation mode")

        # Подписки
        rospy.Subscriber('/uav/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/main_camera/image_raw', Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/mission/start', Empty, self.mission_start_callback)
        rospy.Subscriber('/mission/land', Empty, self.mission_land_callback)
        rospy.Subscriber('/mission/kill', Empty, self.mission_kill_callback)

        # Публикации
        self.branches_pub = rospy.Publisher('/detected_branches', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=10)

        rospy.loginfo("Pipeline inspector node initialized")

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        position = msg.pose.position
        rospy.logdebug(f"Pose: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")

    def mission_start_callback(self, msg):
        rospy.loginfo("Mission start command received")
        self.mission_active = True
        self.detected_branches = []
        self.run_mission()

    def mission_land_callback(self, msg):
        rospy.loginfo("Emergency land command received")
        self.mission_active = False
        self.publish_status("EMERGENCY_LANDING")
        if self.navigate_srv:
            self.navigate_srv(yaw=float('nan'), speed=0.5, frame_id='body', auto_arm=False)
        rospy.sleep(3)
        if self.navigate_srv:
            self.navigate_srv(z=0.0, yaw=float('nan'), speed=0.5, frame_id='body')

    def mission_kill_callback(self, msg):
        rospy.loginfo("Kill switch activated")
        self.mission_active = False
        self.publish_status("KILLED")

    def publish_status(self, status):
        self.status_pub.publish(String(status))

    def image_callback(self, msg):
        if not self.mission_active:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"CV bridge error: {e}")
            return

        branch_positions = self.detect_branches_in_image(cv_image)
        for bp in branch_positions:
            world_pos = self.image_to_world(bp, msg.header)
            if world_pos and self.is_new_branch(world_pos):
                self.detected_branches.append(world_pos)
                rospy.loginfo(f"Branch detected at: x={world_pos.x:.2f}, y={world_pos.y:.2f}")
                self.publish_branches()

    def detect_branches_in_image(self, image):
        """Обнаружение врезок на изображении с камеры дрона."""
        branches = []

        # Конвертация в HSV для обнаружения трубы
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Труба в Gazebo — серый/тёмный цилиндр. Врезки — тоже цилиндры меньшего радиуса.
        # Ищем контуры, которые могут быть врезками.

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Размытие для уменьшения шума
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Адаптивный порог
        thresh = cv2.adaptiveThreshold(
            blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 11, 2
        )

        # Морфологические операции для очистки
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # Поиск контуров
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Фильтрация контуров по размеру — врезка меньше основной трубы
        image_area = image.shape[0] * image.shape[1]
        min_area = image_area * 0.001  # минимальная площадь контура врезки
        max_area = image_area * 0.05   # слишком большие контуры — это сама труба или фон

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if min_area < area < max_area:
                # Проверка формы — врезка ближе к круглой
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                if circularity > 0.3:  # допускаем неидеальные круги
                    # Центр контура
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        branches.append((cx, cy, area))

        return branches

    def image_to_world(self, image_point, header):
        """Перевод координат из изображения в мировые координаты."""
        if not self.current_pose:
            return None

        px, py, area = image_point

        # Простая проекция: камера смотрит вниз
        # Высота дрона определяет масштаб изображения
        drone_z = self.current_pose.position.z
        if drone_z <= 0:
            return None

        # Предполагаем камеру с углом обзора ~60° и разрешением 640x480
        img_width = 640
        img_height = 480
        fov_horizontal = math.radians(60)

        # Размер пикселя в мировых координатах на данной высоте
        scale = (2 * drone_z * math.tan(fov_horizontal / 2)) / img_width

        # Смещение от центра изображения = смещение от позиции дрона
        dx = (px - img_width / 2) * scale
        dy = (py - img_height / 2) * scale

        # Учитываем ориентацию дрона
        q = self.current_pose.orientation
        euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]

        world_x = self.current_pose.position.x + dx * math.cos(yaw) + dy * math.sin(yaw)
        world_y = self.current_pose.position.y - dx * math.sin(yaw) + dy * math.cos(yaw)

        pose = Pose()
        pose.position.x = world_x
        pose.position.y = world_y
        pose.position.z = 0.0  # врезки на уровне земли/трубы

        return pose.position

    def is_new_branch(self, pos):
        """Проверка, что найденная врезка — новая, а не дубль."""
        for bp in self.detected_branches:
            dist = math.sqrt((pos.x - bp.x) ** 2 + (pos.y - bp.y) ** 2)
            if dist < self.min_branch_dist:
                return False
        return True

    def publish_branches(self):
        """Публикация всех обнаруженных врезок."""
        pa = PoseArray()
        pa.header = Header(stamp=rospy.Time.now(), frame_id='map')
        for bp in self.detected_branches:
            pose = Pose()
            pose.position = bp
            pose.orientation = Quaternion(w=1.0)
            pa.poses.append(pose)
        self.branches_pub.publish(pa)

    def load_pipeline_data(self):
        """Загрузка данных о трубопроводе из world-файла."""
        world_path = rospy.get_param('~world_path', '/home/clover/nto_project/oil_pipeline.world')
        try:
            with open(world_path, 'r') as f:
                content = f.read()
            # Парсинг SDF для извлечения сегментов трубы
            # Сегменты трубы — модели с именем pipeline_segment_* или tap_*
            self.parse_world_segments(content)
        except FileNotFoundError:
            rospy.logwarn(f"World file not found: {world_path}")
            self.pipe_segments = []

    def parse_world_segments(self, content):
        """Парсинг SDF-файла для извлечения позиций сегментов трубы."""
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(content)
            self.pipe_segments = []
            for model in root.iter('model'):
                name = model.get('name', '')
                if name.startswith('pipeline_segment') or name.startswith('tap_'):
                    pose_elem = model.find('pose')
                    if pose_elem is not None and pose_elem.text:
                        coords = pose_elem.text.strip().split()
                        x, y = float(coords[0]), float(coords[1])
                        self.pipe_segments.append({
                            'name': name,
                            'x': x,
                            'y': y,
                            'is_tap': name.startswith('tap_')
                        })
        except ET.ParseError:
            rospy.logwarn("Failed to parse world SDF file")

    def run_mission(self):
        """Основная логика миссии."""
        self.publish_status("STARTING")

        # Загрузка данных о трубопроводе
        self.load_pipeline_data()

        if not self.pipe_segments:
            rospy.logwarn("No pipeline data found, attempting search pattern")
            self.search_pattern_mission()
            return

        # Фильтрация: только сегменты основной трубы
        main_segments = [s for s in self.pipe_segments if not s['is_tap']]
        if not main_segments:
            rospy.logwarn("No main pipeline segments found")
            self.publish_status("FAILED")
            return

        # Сортировка сегментов по дистанции от начальной точки
        main_segments.sort(key=lambda s: math.sqrt(s['x'] ** 2 + s['y'] ** 2))

        # Взлёт
        self.publish_status("TAKEOFF")
        start_x = main_segments[0]['x']
        start_y = main_segments[0]['y']
        target_z = self.flight_height

        if self.navigate_global_srv:
            rospy.loginfo(f"Taking off to z={target_z}")
            self.navigate_global_srv(x=start_x, y=start_y, z=target_z, yaw=float('nan'), speed=0.5)
            rospy.sleep(3)

        self.publish_status("FLYING_ALONG_PIPELINE")

        # Полёт вдоль сегментов трубы
        for i, seg in enumerate(main_segments):
            if not self.mission_active:
                return

            rospy.loginfo(f"Flying to segment {i+1}/{len(main_segments)}: ({seg['x']:.2f}, {seg['y']:.2f})")

            if self.navigate_global_srv:
                self.navigate_global_srv(
                    x=seg['x'], y=seg['y'], z=self.flight_height,
                    yaw=float('nan'), speed=0.3
                )
                # Ждём прибытия + время на обработку камеры
                rospy.sleep(4)

        # Полёт к известным позициям врезок для уточнения координат
        taps = [s for s in self.pipe_segments if s['is_tap']]
        if taps and self.mission_active:
            self.publish_status("VERIFYING_BRANCHES")
            for i, tap in enumerate(taps):
                rospy.loginfo(f"Verifying tap {i+1}/{len(taps)}: ({tap['x']:.2f}, {tap['y']:.2f})")
                if self.navigate_global_srv:
                    self.navigate_global_srv(
                        x=tap['x'], y=tap['y'], z=self.flight_height,
                        yaw=float('nan'), speed=0.2
                    )
                    rospy.sleep(3)

        # Финальная публикация всех обнаруженных врезок
        self.publish_branches()

        # Возвращение и посадка
        self.publish_status("RETURNING")
        if self.navigate_global_srv:
            self.navigate_global_srv(x=0, y=0, z=self.flight_height, yaw=float('nan'), speed=0.5)
            rospy.sleep(5)
            self.navigate_global_srv(x=0, y=0, z=0.3, yaw=float('nan'), speed=0.3)
            rospy.sleep(3)

        self.publish_status("COMPLETED")
        rospy.loginfo(f"Mission completed. Found {len(self.detected_branches)} branches.")

    def search_pattern_mission(self):
        """Миссия с паттерном поиска, если данные о трубе неизвестны."""
        self.publish_status("SEARCH_PATTERN")

        # Взлёт в центр
        if self.navigate_global_srv:
            self.navigate_global_srv(x=0, y=0, z=self.flight_height, yaw=float('nan'), speed=0.5)
            rospy.sleep(3)

        # Зигзагообразный паттерн поиска
        search_x_range = 12.0  # максимальная x-координата поиска
        search_y_range = 8.0   # максимальная y-координата поиска
        step = 2.0             # шаг паттерна (м)

        x = 0
        direction = 1

        while x < search_x_range and self.mission_active:
            for y_off in range(int(-search_y_range / step), int(search_y_range / step) + 1):
                y = y_off * step * direction
                if self.navigate_global_srv:
                    self.navigate_global_srv(
                        x=x, y=y, z=self.flight_height,
                        yaw=float('nan'), speed=0.3
                    )
                    rospy.sleep(2)
            x += step
            direction *= -1  # зигзаг

        # Посадка
        if self.navigate_global_srv:
            self.navigate_global_srv(x=0, y=0, z=0.3, yaw=float('nan'), speed=0.3)
            rospy.sleep(3)

        self.publish_status("COMPLETED")


if __name__ == '__main__':
    try:
        inspector = PipelineInspector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
