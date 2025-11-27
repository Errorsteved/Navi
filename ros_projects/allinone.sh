#!/bin/bash

gnome-terminal -- bash -c '
docker start yolov5_container2
export DISPLAY=:1
xhost +local:root
docker exec -it -w /ros2_ws yolov5_container2 bash -c "
    source /opt/ros/foxy/setup.bash
    ./run.sh
"
exec bash
'

gnome-terminal -- bash -c '
docker start yolov5_container2
export DISPLAY=:1
xhost +local:root
docker exec -it -w /ros2_ws yolov5_container2 bash -c "
    source /opt/ros/foxy/setup.bash
    ./tcp_sender.sh
"
exec bash
'

gnome-terminal -- bash -c '
docker start gpsimu_con1
docker exec -it -w /ros2_ws gpsimu_con1 bash -c "
    source /opt/ros/foxy/setup.bash
    ./run.sh
"
exec bash
'

gnome-terminal -- bash -c '
docker start gpsimu_con1
docker exec -it -w /ros2_ws gpsimu_con1 bash -c "
    source /opt/ros/foxy/setup.bash
    ./mqtt.sh
"
exec bash
'

gnome-terminal -- bash -c '
docker start motion_container1
docker exec -it -w /ros2_ws motion_container1 bash -c "
    source /opt/ros/foxy/setup.bash
    ./run.sh
"
exec bash
'

gnome-terminal -- bash -c '
docker start motion_container1
docker exec -it -w /ros2_ws motion_container1 bash -c "
    source /opt/ros/foxy/setup.bash
    ./follower.sh
"
exec bash
'