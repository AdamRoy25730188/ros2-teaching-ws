# ros2-teaching-ws Adam Roy Assessment 1
A template repository provided  for teaching robotics with ROS2 modified to complete CMP3103-2324 assessment item 1

## Use case

You can use this repository to start developing your ROS2 modules. It provides a preconfigured [Development Container](https://containers.dev/) with ROS2 installed, and a VNC based light Desktop integrated directly.

## Usage

This is a fork of the original repository template used to create your own independent repository.

### How to run the enviroment

In order to run the ros2 package and nodes within this repository, after loading it into visual studio using docker, the command:

"pip install setuptools==58.2.0"

Should be used so that colcon may be properly used to build the package. This is done as the version of setuptools and colcon installed by default are depreciated and thus a downpatch to the most recent working version is required. Once done the package may be built using:

"colcon build --packages-select my_pk"

While it is my understanding that there are other and better ways to initialise and build ros2 packages, this is the only method I have found that has worked successfuly. With the package built the enviroment needs to be initialised. First lcas must be logged into from a cmd terminal using:

"docker login -u lcas -p lincoln lcas.lincoln.ac.uk"

Then the tidybot enviroment can be initialised in the novnc virtual enviroment. To do so return to the scr folder integrated terminal and enter:

"ros2 launch uol_tidybot tidybot.launch.py"

And spawn green boxes to be pusshed using the given command:

"ros2 run  uol_tidybot generate_objects --ros-args -p n_objects:=10"

From there the main node/program "colour_chaser" needs to be ran. To do so open a new integrated terminal and use:

"ros2 run my_pk colour_chaser"

To begin the tidybot program. The opencv_bridge node can also be ran in order to get a perspective from the simulated limo bot:

"ros2 run my_pk opencv_bridge"
