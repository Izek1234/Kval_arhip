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

ARUCO_MAP = "".join(
    f"{id_} 0.22 {id_ % 10:.1f} {9 - id_ // 10:.1f} 0 0 0\n"
    for id_ in range(100) if id_ != 17
)

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


def _place_contamination(panel_x, panel_y, existing, cont_size=0.08, min_gap=0.02, max_attempts=200):
    half_panel = 0.2
    min_dist = cont_size + min_gap
    for _ in range(max_attempts):
        cx = panel_x + random.uniform(-half_panel, half_panel)+0.15
        cy = panel_y + random.uniform(-half_panel, half_panel)
        collision = False
        for ex, ey in existing:
            if math.sqrt((cx - ex) ** 2 + (cy - ey) ** 2) < min_dist:
                collision = True
                break
        if not collision:
            return cx, cy
    return None


def create_contamination_sdf(name, cx, cy, z=0.6):
    return f"""<model name="{name}">
  <static>true</static>
  <pose>{cx} {cy} {z:.3f} 0 0 0</pose>
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

    return px-0.25, py + 0.1


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
        existing_cont = []
        for j in range(cont_count):
            pos = _place_contamination(px, py, existing_cont)
            if pos is not None:
                cx, cy = pos
                existing_cont.append((cx, cy))
                models_sdf += create_contamination_sdf(
                    f"contamination_{i+1}_{j+1}", cx, cy
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
