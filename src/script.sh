#!/bin/bash

docker login -u lcas -p lincoln lcas.lincoln.ac.uk

ros2 launch uol_tidybot tidybot.launch.py

ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name m_node my_pk

colcon test

pip install setuptools==58.2.0

colcon build --packages-select my_pk

#green = 0, 150, 0


                 '''
                    following is unfinished attempt to usilise lidar
                    '''
                    '''
                    start_index = max(0, -45)
                    end_index = min(360, 45)

                    xDistFromCentre = abs(400 - cx) / 400


                    rayDepth = self.min_range(self.rays.ranges[start_index:end_index])

                    if(rayDepth < cx):
                        self.get_logger().warning("Wall is somehow in front of box!")
                    if(rayDepth - cx < 0.2 + xDistFromCentre/8): #xDistFromCentre to account for distortion
                        self.get_logger().info(f'Box at {cx}, {cy} against wall')
                        continue
                    self.get_logger().info(f'Box at {cx}, {cy} to be pushed')
                    '''
