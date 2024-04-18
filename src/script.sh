#!/bin/bash

source opt/ros/humble/setup.bash

docker login -u lcas -p lincoln lcas.lincoln.ac.uk

ros2 launch uol_tidybot tidybot.launch.py

ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package

ros2 topic

ros2 node
