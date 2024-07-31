#!/bin/bash

docker login -u lcas -p lincoln lcas.lincoln.ac.uk

ros2 launch uol_tidybot tidybot.launch.py

ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name m_node my_pk

colcon test

pip install setuptools==58.2.0

colcon build --packages-select my_pk

#green = 0, 150, 0