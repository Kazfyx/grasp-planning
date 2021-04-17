# grasp-planning

grasp planning based on obstruction degree

Tested in Ubuntu 16.04, ROS Kinetic

## Download

Clone the repository or manually download the files to your ROS catkin workspace.

## Compile

Install dependency CGAL 4.7
```bash
$ sudo apt-get install libcgal-dev
```
Then you can compile with ROS "catkin_make" command.

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
