#!/usr/bin/env python3
"""
Генерация Gazebo-мира с солнечными панелями для задачи «Инспекция солнечной фермы».
"""

import os
import random
import math
import re
import json
import zipfile
import gzip

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

HEAT_COLORS_RGB = {
    'yellow': (1.0, 0.9, 0.0),
    'orange': (1.0, 0.5, 0.0),
    'red': (1.0, 0.0, 0.0),
}


def generate_solar_panels(num_panels=5, min_edge_dist=2.0, aruco_margin=0.4):
    panels = []
    panel_size = 1.0
    aruco_positions = [(ix * 1.0, iy * 1.0) for ix in range(10) for iy in range(10)]
    for i in range(num_panels):
        attempts = 0
        while attempts < 1000:
            x = random.uniform(1.0, 8.0)
            y = random.uniform(1.0, 8.0)
            ok = True
            for px, py, _, _, _ in panels:
                dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                if dist - panel_size < min_edge_dist:
                    ok = False
                    break
            if ok:
                for ax, ay in aruco_positions:
                    if math.sqrt((x - ax) ** 2 + (y - ay) ** 2) < aruco_margin:
                        ok = False
                        break
            if ok:
                heat_color = random.choice(['yellow', 'orange', 'red'])
                heat_state = HEAT_STATES[heat_color]
                contamination_count = random.randint(2, 5)
                panels.append((x, y, heat_color, heat_state, contamination_count))
                break
            attempts += 1
    return panels


def create_solar_panel_sdf(name, x, y):
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{x} {y} 0.0 1.55 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <mesh>
          <uri>model://solar_panel/meshes/solar_panel.obj</uri>
          <scale>0.5 0.5 0.5</scale>
        </mesh>
      </geometry>
    </visual>
    <collision name="collision">
      <geometry>
        <box><size>1 1 0.1</size></box>
      </geometry>
    </collision>
  </link>
</model>"""


def create_indicator_platform_sdf(name, x, y, color_rgb):
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
        <ambient>{r} {g} {b} 1</ambient>
        <diffuse>{r} {g} {b} 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""


def create_contamination_sdf(name, panel_x, panel_y):
    cx = panel_x + random.uniform(-0.35, 0.35)
    cy = panel_y + random.uniform(-0.35, 0.35)
    z = (cx - panel_x) * math.sin(0.8) + 0.06
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{cx} {cy} {z:.3f} 0 0.8 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box><size>0.08 0.08 0.01</size></box>
      </geometry>
      <material>
        <ambient>0.0 0.8 0.0 1</ambient>
        <diffuse>0.0 0.8 0.0 1</diffuse>
      </material>
    </visual>
  </link>
</model>"""


def _indicator_pos_right(px, py, aruco_positions, aruco_margin=0.4):
    ind_half = 0.15
    min_dist = aruco_margin + ind_half
    # Смещения по Y (справа от панели) и X — приоритет справа (+Y)
    offsets_y = [0.55, 0.45, 0.65, 0.75, 0.35, 0.85, 0.95, 1.05]
    offsets_x = [0.0, -0.15, 0.15, -0.3, 0.3, -0.4, 0.4]
    for dy in offsets_y:
        for dx in offsets_x:
            ind_x = px + dx
            ind_y = py + dy
            collision = False
            for ax, ay in aruco_positions:
                if math.sqrt((ind_x - ax) ** 2 + (ind_y - ay) ** 2) < min_dist:
                    collision = True
                    break
            if not collision:
                return ind_x, ind_y
    # Fallback: слева от панели (-Y)
    for dy in [-0.55, -0.45, -0.65]:
        for dx in offsets_x:
            ind_x = px + dx
            ind_y = py + dy
            collision = False
            for ax, ay in aruco_positions:
                if math.sqrt((ind_x - ax) ** 2 + (ind_y - ay) ** 2) < min_dist:
                    collision = True
                    break
            if not collision:
                return ind_x, ind_y
    return px, py + offsets_y[0]


def generate_world(template_path, output_path, panels, aruco_positions=None):
    if aruco_positions is None:
        aruco_positions = [(ix * 1.0, iy * 1.0) for ix in range(10) for iy in range(10)]
    with open(template_path, 'r') as f:
        content = f.read()

    models_sdf = ""
    for i, (px, py, heat_color, heat_state, cont_count) in enumerate(panels):
        models_sdf += create_solar_panel_sdf(f"panel_{i+1}", px, py) + "\n"
        ind_x, ind_y = _indicator_pos_right(px, py, aruco_positions)
        models_sdf += create_indicator_platform_sdf(
            f"indicator_{i+1}", ind_x, ind_y, HEAT_COLORS_RGB[heat_color]
        ) + "\n"
        for j in range(cont_count):
            models_sdf += create_contamination_sdf(
                f"contamination_{i+1}_{j+1}", px, py
            ) + "\n"

    if '</world>' in content:
        content = content.replace('</world>', models_sdf + '</world>')
    elif '</sdf>' in content:
        content = content.replace('</sdf>', models_sdf + '</sdf>')
    else:
        content = '<sdf version="1.6">\n<world name="solar_farm">\n' + content + models_sdf + '\n</world>\n</sdf>'

    with open(output_path, 'w') as f:
        f.write(content)

    data_path = os.path.join(os.path.dirname(output_path), 'panels_data.json')
    with open(data_path, 'w') as f:
        json.dump(panels, f)
    with open('panels_data.json', 'w') as f:
        json.dump(panels, f)

    return panels


def setup_aruco_map(user_name='clover'):
    map_dir = f"/home/{user_name}/catkin_ws/src/clover/aruco_pose/map"
    os.makedirs(map_dir, exist_ok=True)
    with open(os.path.join(map_dir, 'SolarFarm.txt'), 'w') as f:
        f.write(ARUCO_MAP)
    with open('SolarFarm.txt', 'w') as f:
        f.write(ARUCO_MAP)


def setup_launch_files(user_name='clover'):
    def replace_in_file(filepath, old, new):
        if not os.path.exists(filepath):
            return
        with open(filepath, 'r') as f:
            content = f.read()
        content = content.replace(old, new, 1)
        with open(filepath, 'w') as f:
            f.write(content)

    base = f"/home/{user_name}/catkin_ws/src/clover"

    aruco_launch = f"{base}/clover/launch/aruco.launch"
    replace_in_file(aruco_launch, '<arg name="aruco_detect"', '<arg name="aruco_detect" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="aruco_map"', '<arg name="aruco_map" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="aruco_vpe"', '<arg name="aruco_vpe" default="true"/>')
    replace_in_file(aruco_launch, '<arg name="placement"', '<arg name="placement" default="floor"/>')
    replace_in_file(aruco_launch, '<arg name="length"', '<arg name="length" default="0.22"/>')
    replace_in_file(aruco_launch, '<arg name="map"', '<arg name="map" default="SolarFarm.txt"/>')

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

    sim_launch = f"{base}/clover_simulation/launch/simulator.launch"
    replace_in_file(sim_launch, '<arg name="type"', '<arg name="type" default="gazebo"/>')
    replace_in_file(sim_launch, '<arg name="vehicle"', '<arg name="vehicle" default="clover"/>')
    replace_in_file(sim_launch, '<arg name="main_camera"', '<arg name="main_camera" default="true"/>')
    replace_in_file(sim_launch, '<arg name="rangefinder"', '<arg name="rangefinder" default="true"/>')
    replace_in_file(sim_launch, '<arg name="led"', '<arg name="led" default="true"/>')
    replace_in_file(sim_launch, '<arg name="gps"', '<arg name="gps" default="false"/>')
    replace_in_file(sim_launch, '<arg name="gui"', '<arg name="gui" default="true"/>')


def extract_solar_panel_model(zip_path, target_dir):
    os.makedirs(target_dir, exist_ok=True)
    if not os.path.exists(zip_path):
        print(f"WARNING: {zip_path} не найден!")
        return False
    with zipfile.ZipFile(zip_path, 'r') as z:
        for item in z.namelist():
            rel_path = item.replace('home/clover/Desktop/solar_panel/', '')
            if not rel_path:
                continue
            if item.endswith('/'):
                os.makedirs(os.path.join(target_dir, rel_path), exist_ok=True)
                continue
            target = os.path.join(target_dir, rel_path)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            if item.endswith('.gz'):
                data = gzip.decompress(z.read(item))
                with open(target[:-3], 'wb') as f:
                    f.write(data)
            else:
                with open(target, 'wb') as f:
                    f.write(z.read(item))
    print(f"Модель solar_panel распакована в {target_dir}")
    return True


if __name__ == '__main__':
    import sys
    user_name = sys.argv[1] if len(sys.argv) > 1 else 'clover'

    nto_dir = "/home/clover/Documents/Kval_arhip"
    os.makedirs(nto_dir, exist_ok=True)

    # Распаковка модели solar_panel
    gazebo_model_dir = "/home/clover/catkin_ws/src/sitl_gazebo/models"
    solar_panel_dir = os.path.join(gazebo_model_dir, "solar_panel")
    zip_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'solar_panel.zip')
    extract_solar_panel_model(zip_path, solar_panel_dir)

    # Gazebo env
    env_path = os.path.join(nto_dir, 'set_env.sh')
    with open(env_path, 'w') as f:
        f.write(f'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{gazebo_model_dir}\n')

    template_path = f"/home/{user_name}/catkin_ws/src/clover/clover_simulation/resources/worlds/clover_aruco.world"
    output_path = f"{nto_dir}/solar_farm.world"

    setup_aruco_map(user_name)
    setup_launch_files(user_name)

    panels = generate_solar_panels()
    generate_world(template_path, output_path, panels)

    # Обновление world_name в simulator.launch
    sim_launch = f"/home/{user_name}/catkin_ws/src/clover/clover_simulation/launch/simulator.launch"
    if os.path.exists(sim_launch):
        with open(sim_launch, 'r') as f:
            content = f.read()
        content = re.sub(
            r'<arg name="world_name" value="[^"]*"',
            f'<arg name="world_name" value="/home/clover/Documents/Kval_arhip/solar_farm.world"',
            content
        )
        with open(sim_launch, 'w') as f:
            f.write(content)

    print(f"Мир сгенерирован: {output_path}")
    print(f"Панели:")
    for i, (x, y, hc, hs, cc) in enumerate(panels):
        print(f"  Панель {i+1}: ({x:.2f}, {y:.2f}), {hs}, загрязнений: {cc}")
