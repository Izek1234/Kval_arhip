#!/bin/bash
# Запуск проекта «Инспекция солнечной фермы» — НТО Летающая робототехника

PROJECT_DIR=$(pwd)

# Копирование simulator.launch
cp $PROJECT_DIR/simulator.launch ~/catkin_ws/src/clover/clover_simulation/launch/simulator.launch

# Настройка launch-файлов и генерация мира
python3 scripts/gen_solar_farm.py

# Запуск симулятора
roslaunch clover simulator.launch &

# Ожидание инициализации
sleep 10

# Запуск rosbridge для веб-интерфейса
roslaunch rosbridge_server rosbridge_websocket.launch &

sleep 3

# Запуск ноды инспекции солнечной фермы
python3 scripts/solar_inspector.py &

echo "=== Все ноды запущены ==="
echo "Веб-интерфейс: откройте scripts/solar_farm_frontend.html в браузере"
echo "Для старта миссии нажмите СТАРТ МИССИИ в веб-интерфейсе"
