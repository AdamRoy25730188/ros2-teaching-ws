#!/bin/bash

source opt/ros/humble/setup.bash

docker login -u lcas -p lincoln lcas.lincoln.ac.uk

ros2 launch uol_tidybot tidybot.launch.py

ros2 topic

ros2 node
