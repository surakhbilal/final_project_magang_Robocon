JALANIN SIMULATOR:	
cd main/final_project
source install/setup.bash
ros2 launch ./src/itsr_final_project/launch/bringup.launch.py   
 
JALANIN CONTROLLER Keyboard TELEOP:	
source install/setup.bash	
ros2 run robot_features keyboard_control
(NOTE: Jalankan kontrol robot 1 persatu agar tidak error)

JALANIN UI controller joystick:
cd src/itsr_final_project/robot_features/control_ui
python3 -m http.server 8000
(DEBUGGING SERVER ERROR){
	kill -9 12345
	pkill -f "http.server 8000"
	sudo lsof -i :8000
   ---abistu ulangin lagi jalanin server html---
}
ros2 run robot_features foxglove_counter
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
(NOTE: Jalankan kontrol robot 1 persatu agar tidak error)

JALANIN SEMI_AUTO_PID_CONTROLLER:
ros2 run robot_features semi_auto_controller
ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: -0.7, y: 0.0}"

JALANIN Object detector:
ros2 run robot_features object_detector







