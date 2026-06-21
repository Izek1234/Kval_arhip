#!/bin/bash
# Запуск проекта «Инспекция солнечной фермы» — НТО Летающая робототехника

PROJECT_DIR=$(pwd)
LOG_DIR="$PROJECT_DIR/logs"
mkdir -p "$LOG_DIR"

# Функция корректной остановки всех процессов
cleanup() {
    echo ""
    echo "=== Остановка всех процессов ==="
    # Закрываем tmux-сессии
    tmux kill-session -t simulator 2>/dev/null || true
    tmux kill-session -t rosbridge 2>/dev/null || true
    tmux kill-session -t inspector 2>/dev/null || true
    # На всякий случай убиваем процессы
    pkill -9 -f "roslaunch clover simulator.launch" 2>/dev/null || true
    pkill -9 -f "rosbridge_websocket.launch" 2>/dev/null || true
    pkill -0 9090
    pkill -9 -f "solar_inspector.py" 2>/dev/null || true
    sleep 2
    echo "=== Все процессы остановлены ==="
    exit 0
}

# trap cleanup SIGINT SIGTERM EXIT

echo "=== 1. Убиваем старые процессы ==="
echo ""
echo "=== Остановка всех процессов ==="
    # Закрываем tmux-сессии
tmux kill-session -t simulator 2>/dev/null || true
tmux kill-session -t rosbridge 2>/dev/null || true
tmux kill-session -t inspector 2>/dev/null || true
    # На всякий случай убиваем процессы
pkill -9 -f "roslaunch clover simulator.launch" 2>/dev/null || true
pkill -9 -f "rosbridge_websocket.launch" 2>/dev/null || true
pkill -0 9090
pkill -9 -f "solar_inspector.py" 2>/dev/null || true
sleep 2
echo "=== Все процессы остановлены ==="
killall -9 gzserver gzclient rosmaster rosout roslaunch 2>/dev/null || true
pkill -9 -f "solar_inspector.py" 2>/dev/null || true
pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
fuser -k 9090/tcp 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
fuser -k 11311/tcp 2>/dev/null || true
sudo rm -f /tmp/px4_lock-* /tmp/px4-sock-* 2>/dev/null || true
# Закрываем старые screen-сессии
screen -S simulator -X quit 2>/dev/null || true
screen -S rosbridge -X quit 2>/dev/null || true
screen -S inspector -X quit 2>/dev/null || true
sleep 2

echo "=== 2. Настройка и генерация мира ==="
python3 scripts/gen_solar_farm.py

cp -r $PROJECT_DIR/home/clover/Desktop/solar_panel/ /home/clover/catkin_ws/src/sitl_gazebo/models/
cp $PROJECT_DIR/simulator.launch /home/clover/catkin_ws/src/clover/clover_simulation/launch/simulator.launch

echo "=== 3. Запуск симулятора ==="
source /home/clover/catkin_ws/devel/setup.bash
# Запускаем в tmux-сессии
tmux new-session -d -s simulator "roslaunch clover simulator.launch 2>&1 | tee $LOG_DIR/simulator.log"
sleep 15
SIM_PID=$(pgrep -f "roslaunch clover simulator.launch" | head -1)
echo "Симулятор запущен в tmux-сессии 'simulator' (PID: $SIM_PID)"
echo "Для подключения: tmux attach -t simulator"
echo "Для отключения: Ctrl+B, затем D"

echo "=== 4. Запуск rosbridge ==="
tmux new-session -d -s rosbridge "roslaunch rosbridge_server rosbridge_websocket.launch 2>&1 | tee $LOG_DIR/rosbridge.log"
sleep 3
ROSBRIDGE_PID=$(pgrep -f "rosbridge_websocket.launch" | head -1)
echo "Rosbridge запущен в tmux-сессии 'rosbridge' (PID: $ROSBRIDGE_PID)"

# echo "=== 5. Запуск ноды инспекции ==="
# tmux new-session -d -s inspector "python3 scripts/solar_inspector.py 2>&1 | tee $LOG_DIR/inspector.log"
# sleep 1
# INSPECTOR_PID=$(pgrep -f "solar_inspector.py" | head -1)
# echo "Инспектор запущен в tmux-сессии 'inspector' (PID: $INSPECTOR_PID)"

echo ""
echo "========================================="
echo "=== Все ноды запущены успешно ==="
echo "========================================="
echo ""
echo "📺 Tmux-сессии:"
echo "   simulator  — симулятор Gazebo + PX4"
echo "   rosbridge  — ROS bridge WebSocket"
echo "   inspector  — нода инспекции"
echo ""
echo "🔍 Подключиться к сессии:"
echo "   tmux attach -t simulator"
echo "   tmux attach -t rosbridge"
echo "   tmux attach -t inspector"
echo ""
echo "📁 Логи: $LOG_DIR/"
echo "   tail -f $LOG_DIR/simulator.log"
echo "   tail -f $LOG_DIR/rosbridge.log"
echo "   tail -f $LOG_DIR/inspector.log"
echo ""
echo "🌐 Веб-интерфейс: откройте scripts/solar_farm_frontend.html в браузере"
echo ""
echo "⏹  Для остановки: нажмите Ctrl+C или выполните:"
echo "   tmux kill-session -t simulator"
echo "   tmux kill-session -t rosbridge"
echo "   tmux kill-session -t inspector"
echo "========================================="