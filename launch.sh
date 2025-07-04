#!/bin/bash

python3 -m http.server 8080 --directory /home/ares/workplace/ros_web &
web_pid=$!

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ros_pid=$!

cleanup(){
    kill -9 "$web_pid" 2>/dev/null
    kill -9 "$ros_pid" 2>/dev/null
    exit 0
}
trap cleanup EXIT
wait -n $web_pid $ros_pid