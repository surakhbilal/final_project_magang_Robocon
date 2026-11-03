#JALANIN SIMULATOR:	
cd main/final_project
source install/setup.bash
ros2 launch ./src/itsr_final_project/launch/bringup.launch.py   
 
 ============================================== TR ============================================== #
 
#JALANIN CONTROLLER Keyboard TELEOP TR:		
ros2 run robot_features keyboard_control
(NOTE: Jalankan kontrol robot 1 persatu agar tidak error)

#JALANIN UI controller joystick TR:
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

#JALANIN Object detector TR:
ros2 run robot_features object_detector

#JALANIN SEMI OTONOM TR:
ros2 run robot_features semi_auto_controller

============================================== BR ============================================== #

#JALANIN CONTROLLER Keyboard TELEOP BR:	
ros2 run br_robot_features keyboard_control

#JALANIN Object detector BR:
ros2 run br_robot_features object_detector

#JALANIN NODE SEMI OTONOM BR:
ros2 run br_robot_features br_semi_auto

#=====================================================================================================================================================================================================#

NGIDE

#JALANIN PATH PLANNER TR:
ros2 run robot_features semi_auto_controller
ros2 run robot_features tr_planner
ros2 action send_goal /tr/place_block tr_robot_interfaces/action/PlaceBlock "{phase: '1&2'}" --feedback

#JALANIN PATH PLANNER BR:
ros2 run br_robot_features br_semi_auto
ros2 run br_robot_features br_planner
ros2 action send_goal /br/build_stack br_robot_interfaces/action/BuildStack "{mode: 'MOVE', phase: '1/2/3/4'}" --feedback

#JALANIN SCRIPT SEMI-OTONOM
cd src/itsr_final_project/launch/
python3 mission_manual.py

# List CONTOH gerakan BR dan TR
1. ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: -1.5, y: -4.0}"
2. ros2 topic pub --once /tr/rotate_by std_msgs/msg/Float64 "{data: 1.5708}"
3. ros2 topic pub --once /tr/gripper_cmd std_msgs/msg/Float64 "{data: 0.0}"
4. ros2 topic pub --once /tr/gripper_cmd std_msgs/msg/Float64 "{data: 0.5}"
5. ros2 topic pub --once /tr/gripper_tip_cmd std_msgs/msg/Float64 "{data: 7.5}"
6. ros2 topic pub --once /tr/gripper_tip_cmd std_msgs/msg/Float64 "{data: 0.0}"
7. ros2 topic pub --once /br/z_axis_cmd std_msgs/msg/Float64 "{data: 0.4}"
8. ros2 topic pub --once /br/z_axis_cmd std_msgs/msg/Float64 "{data: 0.8}"
9. ros2 topic pub --once /br/z_axis_cmd std_msgs/msg/Float64 "{data: 0.0}"

#=====================================================================================================================================================================================================#

#LAPORAN PENGGUNAAN AI
1. https://chat.deepseek.com/share/4gb6051jzxvbb669hq
2. https://chat.deepseek.com/share/cmxh7g74ewqdvzeszn
3. https://chat.deepseek.com/share/9kvmrndxlpxawwh425
4. https://chat.deepseek.com/share/3uu7t99a1tod3xv9i5
5. https://chat.deepseek.com/share/vfod8ko8dyufkccidb
6. https://claude.ai/share/97737a3f-bc09-468f-b0aa-402afb03f358
7. https://claude.ai/share/e9158809-2afb-469d-9739-932d8ddd28cb
8. https://claude.ai/share/d30c2013-e038-4ee8-90c7-2ad604bde01b
maaf mas sebelumnya, dikarenakan saya sudah dari awal magang menyusun FPnya jadi ada beberapa history chat yang menghasilkan kode yang sudah terhapus (terutama claude, karena kenak limit.....). selebihnya saya cuma pakai 2 AI, deepseek + claude saja untuk pengerjaan FP ini.

- untuk protos robot saya sesuaikan lagi karena AI tidk bisa langsung build dari 0 lewat kode smpai jadi bagus
            
            
            
            
            
            
            

