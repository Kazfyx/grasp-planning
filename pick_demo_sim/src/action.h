#ifndef ACTION_H
#define ACTION_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometric_shapes/shape_operations.h>
//#include <pick_msgs/cloud_tran.h>
#include <actionlib/client/simple_action_client.h>
//#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
//#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h>
#include <ros/ros.h>
#include "func.h"


using namespace std;

class Action
{
public:
    ros::NodeHandle node_handle;
    moveit::planning_interface::MoveGroupInterface manipulator;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    //pick_msgs::cloud_tran srv;
    geometry_msgs::Pose current_pose;
    ros::Publisher hand_pub;
    bool success=false;
    vector<double> target_joints1={0,0,0,0,0,0};
    vector<double> target_joints2={0,0,0,0,0,0};
    vector<double> target_joints3={0,0,0,0,0,0};
    vector<double> target_joints4={0,0,0,0,0,0};
    vector<double> zero_values={0,0,0,0,0,0};  // + + - - - +
    vector<double> open_values={0.3,0.3,-0.3,-0.3,-0.3,0.3};
    vector<double> close_values={0.7,0.7,-0.7,-0.7,-0.7,0.7};
    bool unknown=false;
    vector<moveit_msgs::CollisionObject> collisions, arrows;
    vector<string> del_name, all_name, arrow_name;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup *arm_model_group, *hand_model_group;
    ros::ServiceClient client;
    geometry_msgs::Pose forward, backward;
    //robotiq_2f_gripper_control::Robotiq2FGripper_robot_output hand_active, hand_close, hand_open, hand_reset;
    std_msgs::Float64MultiArray hand_active, hand_close, hand_open, hand_reset, hand_open2, hand_open3, hand_open4;
    moveit_msgs::CollisionObject ground;
    moveit_msgs::Constraints constraints;
    void get_current_pose();
    array<geometry_msgs::Pose, 5> place_0_pose;
    vector<geometry_msgs::Quaternion> place_orientation;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

public:
    enum {NONE = 0, PICK = 1, PUSH = 2, PLACE = 3};
    vector<obj_grasp> obj_list, obj_list_0;
    vector<place_plan> place_list;
    vector<pick_plan> pick_list;
    vector<push_plan> push_list;
    string inhand_name;
    vector<int> find_id,discover_id;
    vector<int> feasible_act;
    array<int, 5> placeable;
    bool real_plan, pick_target;
    string solver_type;
    int rounds=0;
    int exist_obj_num=0;
    double action_time=0;
    double planning_time=0;
    int motion_planning_trials=0;
    double motion_planning_time=0;
    double analysis_time=0;
    int fallen_num=0;
    map<string, geometry_msgs::Pose> name_pose;

    Action();
    ~Action();
    void file_arrangement();
    void random_arrangement();
    void file_record();
    bool initialize();
    bool look_action();   
    bool plan_pick(vector<grasp_data> &grasp_list, pick_plan &pp);
    bool pick_action(pick_plan &pp);
    bool plan_push_action(int id);
    bool plan_push();
    bool push_action(int id);
    bool plan_place(bool real);
    bool place_action(int x, int y);
    bool plan_place_action(int x, int y);
    bool pick_object_1(int &num);
    bool pick_object_2(int &num);
    bool grasp_planning();

};

#endif
