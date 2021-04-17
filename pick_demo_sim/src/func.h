#ifndef FUNC_H
#define FUNC_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

using namespace std;
const string arm_group_name="manipulator";
const string hand_group_name="gripper";
const string base_link="base_link";
const string tip_link="gripper_tip";
const string camera_link="camera_rgb_optical_frame";
const vector<string> finger_links{"left_outer_finger","left_inner_knuckle","left_inner_finger","right_outer_finger","right_inner_knuckle","right_inner_finger"};
const std::map<std::string, int> name_id{{"wangwang",0}, {"weiquan",1}, {"binggan",2}, {"leshi",3}, {"mofang",4}, {"dingshuji",5}, {"unknown",6}};
const std::map<int, std::string> id_name{{0,"wangwang"}, {1,"weiquan"}, {2,"binggan"}, {3,"leshi"}, {4,"mofang"}, {5,"dingshuji"}, {6,"unknown"}};
const std::map<std::string, std::string> name_shape{{"wangwang","cylinder"},{"weiquan","cylinder"},{"binggan","cylinder"},{"leshi","cylinder"},{"mofang","box"},{"dingshuji","box"}};
const std::map<std::string, double> name_height{{"wangwang",0.09}, {"weiquan",0.135}, {"binggan",0.165}, {"leshi",0.206}, {"mofang",0.06}, {"dingshuji",0.14}};
const double table_height=0.02;

geometry_msgs::Pose TransformToPose(const geometry_msgs::Transform &trans);

void set_group_config(moveit::planning_interface::MoveGroupInterface &group);
bool movePath(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Pose &pose,
              const std::string &pose_reference_frame, bool avoid_collisions=true);
bool planPath(moveit::planning_interface::MoveGroupInterface &group, robot_state::RobotStatePtr &start_state, const geometry_msgs::Pose &pose,
              const std::string &pose_reference_frame, moveit::planning_interface::MoveGroupInterface::Plan &plan, bool avoid_collisions=true);
bool movePose(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Pose &pose, const std::string &pose_reference_frame);
bool movePoint(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Point &point, const std::string &pose_reference_frame);
bool moveJoints(moveit::planning_interface::MoveGroupInterface &group, const std::vector< double > &joints);

struct grasp_data
{
    std::string name;
    geometry_msgs::Pose grasp_pose;  
    double easiness=0;
    std::map<std::string, double> occlusion;
    double total_occ=0;
    bool operator<(const grasp_data &gd) const
    {
        return (easiness-total_occ) > (gd.easiness-gd.total_occ);
    }
};

struct obj_grasp
{
    std::string name;
    int id=0;
    geometry_msgs::Pose obj_pose;
    std::map<std::string, double> occlusion={{"unknown",0}};
    std::vector<grasp_data> grasp_list;    
    double size[3]={0,0,0};
    int find=0;
    int occlusive=0;
    int graspable=0;
    bool operator<(const obj_grasp &og) const
    {
        tf2::Vector3 v(0, 0, 0.15);
        tf2::Transform pose1,pose2;
        tf2::fromMsg(obj_pose, pose1);
        tf2::fromMsg(og.obj_pose, pose2);
        return pose1.getOrigin().distance(v) < pose2.getOrigin().distance(v);
    }
};

struct place_plan
{
    int x,y;
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    place_plan(int _x,int _y):x(_x),y(_y){}
};

struct pick_plan
{
    int id;
    std::string name;
    moveit::planning_interface::MoveGroupInterface::Plan plan1,plan2;
    pick_plan(int _id, std::string _name):id(_id), name(_name){}
};

struct push_plan
{
    int id;
    geometry_msgs::Pose push_start;
    moveit::planning_interface::MoveGroupInterface::Plan plan1,plan2;
    push_plan(int _id):id(_id){}
};

moveit_msgs::CollisionObject createCollisionObject(obj_grasp &og);
bool next_combination(std::vector<bool> &index);
bool swap_back(std::vector<int> &vec, int index);

#endif
