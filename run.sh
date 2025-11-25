#!/bin/bash

gnome-terminal --tab --title="roscore" \
    -- bash -c "roscore; exec bash"

sleep 1

gnome-terminal --tab --title="rosbridge" \
    -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"

sleep 1

gnome-terminal --tab --title="Hand Gesture App" \
    -- bash -c "cd ~/RoboticArmControlHandGesture/hand-gesture-recognition-mediapipe/ && source .venv/bin/activate && python app.py; exec bash"

sleep 1

gnome-terminal --tab --title="MoveIt Sim" \
    -- bash -c "cd ~/RoboticArmControlHandGesture/moveit_ws/ && source devel/setup.bash && roslaunch movit_robot_arm_sim full_robot_arm_sim.launch; exec bash"

sleep 1

gnome-terminal --tab --title="Set Predefined Pose" \
    -- bash -c "cd ~/RoboticArmControlHandGesture/moveit_ws/ && source devel/setup.bash && rosrun movit_robot_arm_sim node_set_predefined_pose.py; exec bash"

