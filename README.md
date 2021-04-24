# grasp-planning

grasp planning based on obstruction degree

Tested in Ubuntu 16.04, ROS Kinetic

## Prerequisites

ROS "Desktop Full" install: [instructions](http://wiki.ros.org/ROS/Installation).

Install dependencies
```bash
$ sudo apt-get install ros-kinetic-moveit
$ sudo apt-get install ros-kinetic-industrial-core
$ sudo apt-get install ros-kinetic-abb
$ sudo apt-get install libcgal-dev
```

## Compile

Clone the repository or manually download the files to your ROS catkin workspace and compile them with "catkin_make" command: [instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

## Usage

1. Start gazebo simulator
```bash
$ roslaunch my_abb_support my_abb_gazebo.launch
```

2. Start MoveIt motion planning in a new terminal
```bash
$ roslaunch my_abb_moveit_config my_abb_moveit_gazebo.launch
```

3. Open a new termianl, load the object models and start the grasp planning program
```bash
$ roslaunch pick_demo_sim spawn_objects.launch
$ rosrun pick_demo_sim pick_demo_sim
```
