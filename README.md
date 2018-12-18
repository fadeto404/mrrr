# mrrr
**Mine Rescue Reconnaissance Robot (MRRR)**

Repository for AAU C2-19 P1 - Mine Rescue Reconnaissance Robot. Intended for use with a Turtlebot2 running ROS Kinetic.

## Installation of package
Clone this repo where you want your ROS workspace. Enter the folder using ```cd mrrr/```, then build it by running ```catkin_make```. Then run the command ```source devel/setup.bash```.

## Usage of package
To use the package it needs to be installed on both the computer running the TurtleBot2 and a remote PC.

### Running nodes
On the Turtlebot computer, run the following command: ```roslaunch mine_explorer bot.launch```, and on the remote pc run ```roslaunch mine_explorer operator.launch```.

Be aware that the autopilot-node has to be run independently of the launch-files. This is done using either ```rosrun mine_explorer autopilot``` or alternatively ```python </path/to/script>```.
