#!/bin/bash
# Запуск проекта «Инспекция солнечной фермы» — НТО Летающая робототехника

set -e

PROJECT_DIR=$(pwd)

echo "=== 1. Убиваем старые процессы ==="
# Убиваем все ROS/Gazebo процессы от предыдущего запуска
killall -9 gzserver gzclient rosmaster rosout roslaunch rosbridge_websocket python3 2>/dev/null || true
# Убиваем процесс на порту 9090 (rosbridge)
fuser -k 9090/tcp 2>/dev/null || true
sleep 2

echo "=== 2. Копирование simulator.launch ==="
# cp $PROJECT_DIR/simulator.launch ~/catkin_ws/src/clover/clover_simulation/launch/simulator.launch

echo "=== 3. Настройка и генерация мира ==="
python3 scripts/gen_solar_farm.py

echo "=== 4. Запуск симулятора ==="
source /home/clover/catkin_ws/devel/setup.bash
roslaunch clover simulator.launch &
SIM_PID=$!

echo "=== 5. Ожидание инициализации симулятора ==="
sleep 15

echo "=== 6. Запуск rosbridge ==="
roslaunch rosbridge_server rosbridge_websocket.launch &
ROSBRIDGE_PID=$!
sleep 3

echo "=== 7. Запуск ноды инспекции ==="
python3 scripts/solar_inspector.py &
INSPECTOR_PID=$!

echo "=== Все ноды запущены ==="
echo "PID симулятора: $SIM_PID"
echo "PID rosbridge: $ROSBRIDGE_PID"
echo "PID инспектора: $INSPECTOR_PID"
echo ""
echo "Веб-интерфейс: откройте scripts/solar_farm_frontend.html в браузере"
echo "Для старта миссии нажмите СТАРТ МИССИИ в веб-интерфейсе"
echo ""
echo "Для остановки всех процессов: kill $SIM_PID $ROSBRIDGE_PID $INSPECTOR_PID"
