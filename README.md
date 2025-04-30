# chess_robot
## Requirements
- `ubuntu 20.04`

For running each sample code:
- `Ros Noetic:` http://wiki.ros.org/noetic/Installation
- `Gazebo:` https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros
- `Catkin` https://catkin-tools.readthedocs.io/en/latest/
## ROS Distributed Communication
https://www.yahboom.com/build.html?id=10510&cid=648
## cobot280
### Terminal 1
```bash
ros core
```
### Terminal 2
```bash
rosrun jetcobot_moveit gripper_service.py
```
## Host
### Terminal 1
```bash
source devel/setup.bash
roslaunch jetcobot_moveit_config demo.launch
```
### Terminal 2
```bash
source devel/setup.bash
rosrun chess_ai chess_ai_node.py
```
### Terminal 3
```bash
source devel/setup.bash
rosrun chessrobot_simulation pose_receiver
```
### Terminal 4
```bash
source devel/setup.bash
rosrun robot_service robot_service.py
```
### Terminal 5
```bash
source devel/setup.bash
rostopic pub /chess_notation_topic std_msgs/String "data: 'start'" 
```
