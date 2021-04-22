gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch realsense2_camera rs_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/openvino_ws/devel/setup.bash; roslaunch human_pose pose.launch; exec bash"' \

