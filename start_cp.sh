sudo apt install screen
sudo apt install tmux
echo "=== 2. Копирование simulator.launch ==="
cp -r $PROJECT_DIR/home/clover/Desktop/solar_panel/ /home/clover/catkin_ws/src/sitl_gazebo/models/
cp $PROJECT_DIR/simulator.launch /home/clover/catkin_ws/src/clover/clover_simulation/launch/simulator.launch