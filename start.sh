cp $(pwd)/simulator.launch ~/catkin_ws/src/clover/clover_simulation/launch/simulator.launch
python3 scripts/gen_world.py

# Запуск симулятора
roslaunch clover simulator.launch &

# Ожидание инициализации ROS
sleep 10

# Запуск rosbridge для веб-интерфейса
roslaunch rosbridge_server rosbridge_websocket.launch &

# Ожидание rosbridge
sleep 3

# Запуск ноды инспектора трубопровода
python3 scripts/pipeline_inspector.py &

echo "=== Все ноды запущены ==="
echo "Веб-интерфейс: откройте Front-end_drone_v4.html в браузере"
echo "Для старта миссии нажмите СТАРТ МИССИИ в веб-интерфейсе" 
