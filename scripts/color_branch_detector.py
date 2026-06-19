#!/usr/bin/env python3
"""
Нода цветового обнаружения трубопровода и врезок.
Использует цветовые фильтры для выделения трубы и врезок на изображении.
"""

import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import tf.transformations as tft


class ColorBasedDetector:
    """Обнаружение врезок по цветовым признакам на изображении."""

    def __init__(self):
        rospy.init_node('color_branch_detector', log_level=rospy.INFO)

        self.bridge = CvBridge()
        self.drone_pose = None
        self.branches = []
        self.min_branch_dist = 0.5

        rospy.Subscriber('/uav/pose', PoseStamped, self._pose_cb)
        rospy.Subscriber('/main_camera/image_raw', Image, self._image_cb, queue_size=1)

        self.branch_pub = rospy.Publisher('/detected_branches_color', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher('/mission/status', String, queue_size=10)

        rospy.loginfo("Color-based branch detector initialized")

    def _pose_cb(self, msg):
        self.drone_pose = msg.pose

    def _image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f"CV bridge error: {e}")
            return

        # Уменьшаем размер для ускорения обработки
        small = cv2.resize(frame, (320, 240))

        # Ищем T-образные соединения (врезки) через морфологию
        gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Расширение контуров для соединения близких линий
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilated = cv2.dilate(edges, kernel, iterations=2)

        # Поиск контуров
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        img_area = small.shape[0] * small.shape[1]
        branches_img = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < img_area * 0.005 or area > img_area * 0.15:
                continue

            # Анализ формы: врезка выглядит как T или L
            x, y, w, h = cv2.boundingRect(cnt)
            aspect = w / h if h > 0 else 999

            # Врезка — не круглая, а вытянутая (aspect ratio не ~1)
            if 0.2 < aspect < 5.0:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    branches_img.append((cx, cy))

        # Перевод в мировые координаты
        for (cx, cy) in branches_img:
            # Масштабирование обратно к полному размеру изображения
            cx_full = cx * (frame.shape[1] / 320)
            cy_full = cy * (frame.shape[0] / 240)

            world_pos = self._img_to_world(cx_full, cy_full, msg.header)
            if world_pos and self._is_new(world_pos):
                self.branches.append(world_pos)
                rospy.loginfo(f"Color detector: branch at ({world_pos.x:.2f}, {world_pos.y:.2f})")

        self._publish()

    def _img_to_world(self, px, py, header):
        if not self.drone_pose:
            return None

        z = self.drone_pose.position.z
        if z <= 0:
            return None

        img_w, img_h = 640, 480
        fov = math.radians(60)
        scale = (2 * z * math.tan(fov / 2)) / img_w

        dx = (px - img_w / 2) * scale
        dy = (py - img_h / 2) * scale

        q = self.drone_pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        p = Point()
        p.x = self.drone_pose.position.x + dx * math.cos(yaw) + dy * math.sin(yaw)
        p.y = self.drone_pose.position.y - dx * math.sin(yaw) + dy * math.cos(yaw)
        p.z = 0.0
        return p

    def _is_new(self, pos):
        for b in self.branches:
            if math.sqrt((pos.x - b.x) ** 2 + (pos.y - b.y) ** 2) < self.min_branch_dist:
                return False
        return True

    def _publish(self):
        pa = PoseArray(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        for b in self.branches:
            p = Pose(position=b, orientation=Quaternion(w=1.0))
            pa.poses.append(p)
        self.branch_pub.publish(pa)


if __name__ == '__main__':
    try:
        ColorBasedDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
