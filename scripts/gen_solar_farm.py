#!/usr/bin/env python3
"""
Генерация Gazebo-мира с солнечными панелями для задачи «Инспекция солнечной фермы».
Случайная генерация: 5 панелей, индикационные площадки (перегрев), объекты загрязнения.
"""

import os
import random
import math
import shutil
import xml.etree.ElementTree as ET


# ArUco-карта для навигации
ARUCO_MAP = """# id length x y z rot_z rot_y rot_x
0 0.22 0.0 0.0 0 0 0
1 0.22 1.0 0.0 0 0 0
2 0.22 2.0 0.0 0 0 0
3 0.22 3.0 0.0 0 0 0
4 0.22 4.0 0.0 0 0 0
5 0.22 5.0 0.0 0 0 0
6 0.22 6.0 0.0 0 0 0
7 0.22 7.0 0.0 0 0 0
8 0.22 8.0 0.0 0 0 0
9 0.22 9.0 0.0 0 0 0
10 0.22 0.0 1.0 0 0 0
11 0.22 1.0 1.0 0 0 0
12 0.22 2.0 1.0 0 0 0
13 0.22 3.0 1.0 0 0 0
14 0.22 4.0 1.0 0 0 0
15 0.22 5.0 1.0 0 0 0
16 0.22 6.0 1.0 0 0 0
17 0.22 7.0 1.0 0 0 0
18 0.22 8.0 1.0 0 0 0
19 0.22 9.0 1.0 0 0 0
20 0.22 0.0 2.0 0 0 0
21 0.22 1.0 2.0 0 0 0
22 0.22 2.0 2.0 0 0 0
23 0.22 3.0 2.0 0 0 0
24 0.22 4.0 2.0 0 0 0
25 0.22 5.0 2.0 0 0 0
26 0.22 6.0 2.0 0 0 0
27 0.22 7.0 2.0 0 0 0
28 0.22 8.0 2.0 0 0 0
29 0.22 9.0 2.0 0 0 0
30 0.22 0.0 3.0 0 0 0
31 0.22 1.0 3.0 0 0 0
32 0.22 2.0 3.0 0 0 0
33 0.22 3.0 3.0 0 0 0
34 0.22 4.0 3.0 0 0 0
35 0.22 5.0 3.0 0 0 0
36 0.22 6.0 3.0 0 0 0
37 0.22 7.0 3.0 0 0 0
38 0.22 8.0 3.0 0 0 0
39 0.22 9.0 3.0 0 0 0
40 0.22 0.0 4.0 0 0 0
41 0.22 1.0 4.0 0 0 0
42 0.22 2.0 4.0 0 0 0
43 0.22 3.0 4.0 0 0 0
44 0.22 4.0 4.0 0 0 0
45 0.22 5.0 4.0 0 0 0
46 0.22 6.0 4.0 0 0 0
47 0.22 7.0 4.0 0 0 0
48 0.22 8.0 4.0 0 0 0
49 0.22 9.0 4.0 0 0 0
50 0.22 0.0 5.0 0 0 0
51 0.22 1.0 5.0 0 0 0
52 0.22 2.0 5.0 0 0 0
53 0.22 3.0 5.0 0 0 0
54 0.22 4.0 5.0 0 0 0
55 0.22 5.0 5.0 0 0 0
56 0.22 6.0 5.0 0 0 0
57 0.22 7.0 5.0 0 0 0
58 0.22 8.0 5.0 0 0 0
59 0.22 9.0 5.0 0 0 0
60 0.22 0.0 6.0 0 0 0
61 0.22 1.0 6.0 0 0 0
62 0.22 2.0 6.0 0 0 0
63 0.22 3.0 6.0 0 0 0
64 0.22 4.0 6.0 0 0 0
65 0.22 5.0 6.0 0 0 0
66 0.22 6.0 6.0 0 0 0
67 0.22 7.0 6.0 0 0 0
68 0.22 8.0 6.0 0 0 0
69 0.22 9.0 6.0 0 0 0
70 0.22 0.0 7.0 0 0 0
71 0.22 1.0 7.0 0 0 0
72 0.22 2.0 7.0 0 0 0
73 0.22 3.0 7.0 0 0 0
74 0.22 4.0 7.0 0 0 0
75 0.22 5.0 7.0 0 0 0
76 0.22 6.0 7.0 0 0 0
77 0.22 7.0 7.0 0 0 0
78 0.22 8.0 7.0 0 0 0
79 0.22 9.0 7.0 0 0 0
80 0.22 0.0 8.0 0 0 0
81 0.22 1.0 8.0 0 0 0
82 0.22 2.0 8.0 0 0 0
83 0.22 3.0 8.0 0 0 0
84 0.22 4.0 8.0 0 0 0
85 0.22 5.0 8.0 0 0 0
86 0.22 6.0 8.0 0 0 0
87 0.22 7.0 8.0 0 0 0
88 0.22 8.0 8.0 0 0 0
89 0.22 9.0 8.0 0 0 0
90 0.22 0.0 9.0 0 0 0
91 0.22 1.0 9.0 0 0 0
92 0.22 2.0 9.0 0 0 0
93 0.22 3.0 9.0 0 0 0
94 0.22 4.0 9.0 0 0 0
95 0.22 5.0 9.0 0 0 0
96 0.22 6.0 9.0 0 0 0
97 0.22 7.0 9.0 0 0 0
98 0.22 8.0 9.0 0 0 0
99 0.22 9.0 9.0 0 0 0
"""

HEAT_STATES = {
    'yellow': 'нормальное состояние',
    'orange': 'некритический перегрев',
    'red': 'срочный ремонт',
}

HEAT_COLORS_HSV = {
    'yellow': (0.12, 0.9, 0.9),    # Gazebo HSV для желтой площадки
    'orange': (0.08, 0.9, 0.85),    # оранжевая
    'red': (0.0, 0.9, 0.8),         # красная
}

HEAT_COLORS_RGB = {
    'yellow': (1.0, 0.9, 0.0),
    'orange': (1.0, 0.5, 0.0),
    'red': (1.0, 0.0, 0.0),
}


def generate_solar_panels(num_panels=5, min_edge_dist=2.0):
    """Генерация случайных позиций 5 солнечных панелей с условиями задачи."""
    panels = []
    panel_size = 1.0  # размер панели ~1м (для расчёта расстояния между краями)

    for i in range(num_panels):
        attempts = 0
        while attempts < 1000:
            # Координаты в пределах x ∈ [0,9], y ∈ [0,9] (aruco_map)
            x = random.uniform(1.0, 8.0)
            y = random.uniform(1.0, 8.0)

            # Проверка минимального расстояния между краями панелей >= 2м
            ok = True
            for px, py, _, _, _ in panels:
                dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                edge_dist = dist - panel_size  # расстояние между краями
                if edge_dist < min_edge_dist:
                    ok = False
                    break

            if ok:
                # Случайная степень перегрева
                heat_color = random.choice(['yellow', 'orange', 'red'])
                heat_state = HEAT_STATES[heat_color]

                # Случайное количество загрязнений (2-5)
                contamination_count = random.randint(2, 5)

                panels.append((x, y, heat_color, heat_state, contamination_count))
                break
            attempts += 1

    return panels


def create_indicator_platform_sdf(name, x, y, color_rgb):
    """SDF-модель индикационной площадки (плоский квадрат цвета перегрева)."""
    r, g, b = color_rgb
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{x} {y} 0.05 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.3 0.3 0.02</size></box>
      </geometry>
      <material>
        <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script>
        <ambient>{r} {g} {b} 1</ambient>
        <diffuse>{r} {g} {b} 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""


def create_contamination_sdf(name, x, y, panel_x, panel_y):
    """SDF-модель загрязнения (маленький зелёный квадрат на поверхности панели)."""
    # Загрязнение в пределах поверхности панели (±0.4м от центра)
    cx = panel_x + random.uniform(-0.35, 0.35)
    cy = panel_y + random.uniform(-0.35, 0.35)
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{cx} {cy} 0.1 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.15 0.15 0.01</size></box>
      </geometry>
      <material>
        <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Green</name></script>
        <ambient>0.0 0.8 0.0 1</ambient>
        <diffuse>0.0 0.8 0.0 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""


def create_solar_panel_sdf(name, x, y):
    """SDF-модель солнечной панели (плоский прямоугольник)."""
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{x} {y} 0.08 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box><size>1.0 0.6 0.04</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1.0 0.6 0.04</size></box>
      </geometry>
      <material>
        <script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/DarkBlue</name></script>
      </material>
    </visual>
  </link>
</model>"""


def generate_world(template_path, output_path, panels):
    """Создание Gazebo-мира на основе шаблона с добавлением солнечных панелей."""
    # Чтение шаблонного мира
    with open(template_path, 'r') as f:
        content = f.read()

    # Парсинг SDF
    try:
        root = ET.fromstring(content)
    except ET.ParseError:
        # Если шаблон не SDF, создаём мир с нуля
        root = ET.Element('sdf', version='1.6')
        world = ET.SubElement(root, 'world', name='solar_farm')
        ET.SubElement(world, 'scene')
        ET.SubElement(world, 'physics')
        ET.SubElement(world, 'spherical_coordinates')

        # Солнце
        sun = ET.SubElement(world, 'light', name='sun', type='directional')
        ET.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
        ET.SubElement(sun, 'diffuse').text = '0.8 0.8 0.8 1'
        ET.SubElement(sun, 'specular').text = '0.2 0.2 0.2 1'

        # Земля
        ground = ET.SubElement(world, 'model', name='ground_plane')
        ET.SubElement(ground, 'static').text = 'true'
        link = ET.SubElement(ground, 'link', name='link')
        visual = ET.SubElement(link, 'visual', name='visual')
        ET.SubElement(visual, 'cast_shadows').text = '0'
        geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(geom, 'plane')
        size = ET.SubElement(geom, 'plane')
        ET.SubElement(size, 'normal').text = '0 0 1'
        ET.SubElement(size, 'size').text = '100 100'

    world_elem = root.find('world')
    if world_elem is None:
        world_elem = root

    # Добавление солнечных панелей и сопутствующих объектов
    models_xml = ""
    for i, (px, py, heat_color, heat_state, cont_count) in enumerate(panels):
        # Солнечная панель
        models_xml += create_solar_panel_sdf(f"panel_{i+1}", px, py) + "\n"

        # Индикационная площадка (в пределах 0.5м от панели)
        ind_offset_x = random.uniform(-0.3, 0.3)
        ind_offset_y = random.uniform(0.35, 0.45)
        ind_x = px + ind_offset_x
        ind_y = py + ind_offset_y
        models_xml += create_indicator_platform_sdf(
            f"indicator_{i+1}", ind_x, ind_y, HEAT_COLORS_RGB[heat_color]
        ) + "\n"

        # Объекты загрязнения (2-5 зелёных площадок на панели)
        for j in range(cont_count):
            models_xml += create_contamination_sdf(
                f"contamination_{i+1}_{j+1}", px, py, px, py
            ) + "\n"

    # Вставка моделей в SDF
    for model_str in models_xml.strip().split("</model>"):
        if "<model" in model_str:
            model_str += "</model>"
            try:
                model_elem = ET.fromstring(model_str)
                world_elem.append(model_elem)
            except ET.ParseError:
                pass

    # Запись результата
    tree = ET.ElementTree(root)
    tree.write(output_path, encoding='unicode', xml_declaration=True)

    # Сохранение данных о панелях для использования нодой полёта
    import json
    data_path = os.path.join(os.path.dirname(output_path), 'panels_data.json')
    with open(data_path, 'w') as f:
        json.dump(panels, f)
    # Также в текущую директорию скрипта
    with open('panels_data.json', 'w') as f:
        json.dump(panels, f)

    return panels


def setup_aruco_map(user_name='clover'):
    """Настройка ArUco-карты для навигации."""
    map_dir = f"/home/{user_name}/catkin_ws/src/clover/aruco_pose/map"
    os.makedirs(map_dir, exist_ok=True)

    with open(os.path.join(map_dir, 'SolarFarm.txt'), 'w') as f:
        f.write(ARUCO_MAP)

    # Копирование в текущую директорию тоже
    with open('SolarFarm.txt', 'w') as f:
        f.write(ARUCO_MAP)

    return os.path.join(map_dir, 'SolarFarm.txt')


def setup_launch_files(user_name='clover'):
    """Автоматическая настройка clover.launch и aruco.launch."""

    def replace_in_file(filepath, old, new):
        with open(filepath, 'r') as f:
            content = f.read()
        content = content.replace(old, new, 1)
        with open(filepath, 'w') as f:
            f.write(content)

    base = f"/home/{user_name}/catkin_ws/src/clover"

    # aruco.launch
    aruco_launch = f"{base}/clover/launch/aruco.launch"
    replace_in_file(aruco_launch, '<arg name="aruco_detect"', '<arg name="aruco_detect" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="aruco_map"', '<arg name="aruco_map" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="aruco_vpe"', '<arg name="aruco_vpe" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="placement"', '<arg name="placement" default="floor"/>')
    replace_in_file(aruco_launch, '<arg name="length"', '<arg name="length" default="0.22"/>')
    replace_in_file(aruco_launch, '<arg name="map"', '<arg name="map" default="SolarFarm.txt"/>')

    # clover.launch
    clover_launch = f"{base}/clover/launch/clover.launch"
    replace_in_file(clover_launch, '<arg name="fcu_conn"', '<arg name="fcu_conn" default="usb"/>')
    replace_in_file(clover_launch, '<arg name="fcu_ip"', '<arg name="fcu_ip" default="127.0.0.1"/>')
    replace_in_file(clover_launch, '<arg name="main_camera"', '<arg name="main_camera" default="true"/>')
    replace_in_file(clover_launch, '<arg name="optical_flow"', '<arg name="optical_flow" default="true"/>')
    replace_in_file(clover_launch, '<arg name="aruco"', '<arg name="aruco" default="true"/>')
    replace_in_file(clover_launch, '<arg name="rangefinder_vl53l1x"', '<arg name="rangefinder_vl53l1x" default="true"/>')
    replace_in_file(clover_launch, '<arg name="led"', '<arg name="led" default="true"/>')
    replace_in_file(clover_launch, '<arg name="rosbridge"', '<arg name="rosbridge" default="true"/>')
    replace_in_file(clover_launch, '<arg name="web_video_server"', '<arg name="web_video_server" default="true"/>')

    # simulator.launch
    sim_launch = f"{base}/clover_simulation/launch/simulator.launch"
    replace_in_file(sim_launch, '<arg name="type"', '<arg name="type" default="gazebo"/>')
    replace_in_file(sim_launch, '<arg name="vehicle"', '<arg name="vehicle" default="clover"/>')
    replace_in_file(sim_launch, '<arg name="main_camera"', '<arg name="main_camera" default="true"/>')
    replace_in_file(sim_launch, '<arg name="rangefinder"', '<arg name="rangefinder" default="true"/>')
    replace_in_file(sim_launch, '<arg name="led"', '<arg name="led" default="true"/>')
    replace_in_file(sim_launch, '<arg name="gps"', '<arg name="gps" default="false"/>')
    replace_in_file(sim_launch, '<arg name="gui"', '<arg name="gui" default="true"/>')


if __name__ == '__main__':
    import sys
    user_name = sys.argv[1] if len(sys.argv) > 1 else 'clover'

    # Создание директории nto_project если не существует
    nto_dir = f"/home/Documents/Kval_arhip"
    os.makedirs(nto_dir, exist_ok=True)

    template_path = f"/home/{user_name}/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world"
    output_path = f"{nto_dir}/solar_farm.world"

    # Настройка ArUco и launch-файлов
    setup_aruco_map(user_name)
    setup_launch_files(user_name)

    # Генерация панелей
    panels = generate_solar_panels()

    # Генерация мира
    generate_world(template_path, output_path, panels)

    # Обновление world_name в simulator.launch
    sim_launch = f"/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch"
    with open(sim_launch, 'r') as f:
        content = f.read()
    # Заменить world_name на наш мир
    import re
    content = re.sub(
        r'<arg name="world_name" value="[^"]*"',
        f'<arg name="world_name" value="/home/Documents/Kval_arhip/solar_farm.world"',
        content
    )
    with open(sim_launch, 'w') as f:
        f.write(content)

    print(f"Мир сгенерирован: {output_path}")
    print(f"Панели:")
    for i, (x, y, hc, hs, cc) in enumerate(panels):
        print(f"  Панель {i+1}: ({x:.2f}, {y:.2f}), {hs}, загрязнений: {cc}")
