#include "func.h"
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <geometric_shapes/shape_operations.h>


geometry_msgs::Pose TransformToPose(const geometry_msgs::Transform &trans)
{
    geometry_msgs::Pose pose;
    pose.position.x=trans.translation.x;
    pose.position.y=trans.translation.y;
    pose.position.z=trans.translation.z;
    pose.orientation.x=trans.rotation.x;
    pose.orientation.y=trans.rotation.y;
    pose.orientation.z=trans.rotation.z;
    pose.orientation.w=trans.rotation.w;
    return pose;
}

void set_group_config(moveit::planning_interface::MoveGroupInterface &group)
{
    //ros::param::set("/move_group/trajectory_execution/allowed_execution_duration_scaling", 10);
    //ros::param::set("/move_group/trajectory_execution/execution_duration_monitoring", false);
    //group.setPlannerId("RRTstarkConfigDefault");
    //group.setPlannerId("RRTkConfigDefault");
    //group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPlannerId("PRMkConfigDefault");
    //group.setPlannerId("PRMstarkConfigDefault");
    //group.setNumPlanningAttempts(10);
    group.setPlanningTime(1.0);
    ROS_INFO_STREAM("Planning time: "<<group.getPlanningTime());
    //group.setMaxVelocityScalingFactor(1);
    //ROS_INFO("GoalPositionTolerance: %f", group.getGoalPositionTolerance());
    //ROS_INFO("GoalOrientationTolerance: %f", group.getGoalOrientationTolerance());
    //group.allowLooking(true);
    //group.allowReplanning(true);
    group.setEndEffectorLink(tip_link);
    //ROS_INFO("End Effector Link: %s", group.getEndEffectorLink().c_str());
}

bool movePath(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Pose &pose,
              const std::string &pose_reference_frame, bool avoid_collisions)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints(1, pose);
    group.setPoseReferenceFrame(pose_reference_frame);
    ROS_INFO("Start move path! Reference frame: %s", group.getPoseReferenceFrame().c_str());
    bool flag=false;
    group.setStartStateToCurrentState();
    double fraction = group.computeCartesianPath(waypoints, 0.01, 1.57, trajectory, avoid_collisions);
    if (fraction>0.7)
    {
//        // The trajectory needs to be modified so it will include velocities as well.
//        // First to create a RobotTrajectory object
//        robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
//        // Second get a RobotTrajectory from trajectory
//        rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
//        // Third create a IterativeParabolicTimeParameterization object
//        trajectory_processing::IterativeParabolicTimeParameterization iptp;
//        // Fourth compute computeTimeStamps
//        if (iptp.computeTimeStamps(rt))
//        {
//            // Get RobotTrajectory_msg from RobotTrajectory
//            rt.getRobotTrajectoryMsg(trajectory);
            plan.trajectory_ = trajectory;
            flag=(group.execute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
//        }
//        else
//        {
//            ROS_INFO("Compute time stamp failed");
//            flag=false;
//        }
    }
    else if (fraction<0)
    {
        ROS_WARN("Compute path failed!");
        flag=false;
    }
    else
    {
        ROS_WARN("Bad cartesian path! %f", fraction);
        flag=false;
    }
    sleep(1);
    return flag;
}

bool planPath(moveit::planning_interface::MoveGroupInterface &group, robot_state::RobotStatePtr &start_state, const geometry_msgs::Pose &pose,
              const std::string &pose_reference_frame, moveit::planning_interface::MoveGroupInterface::Plan &plan, bool avoid_collisions)
{
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints(1, pose);
    group.setPoseReferenceFrame(pose_reference_frame);
    ROS_INFO("Start plan path! Reference frame: %s", group.getPoseReferenceFrame().c_str());
    bool flag=false;
    group.setStartState(*start_state);
    double fraction = group.computeCartesianPath(waypoints, 0.01, 1.57, trajectory, avoid_collisions);
    //yougaidong
    if (fraction>0.7)
    {
//        // The trajectory needs to be modified so it will include velocities as well.
//        // First to create a RobotTrajectory object
//        robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group.getName());
//        // Second get a RobotTrajectory from trajectory
//        rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);
//        // Third create a IterativeParabolicTimeParameterization object
//        trajectory_processing::IterativeParabolicTimeParameterization iptp;
//        // Fourth compute computeTimeStamps
//        if (iptp.computeTimeStamps(rt))
//       {
//            // Get RobotTrajectory_msg from RobotTrajectory
//            rt.getRobotTrajectoryMsg(trajectory);
            plan.trajectory_ = trajectory;
            flag=true;
//        }
//        else
//        {
//            ROS_INFO("Compute time stamp failed");
//            flag=false;
//        }
    }
    else if (fraction<0)
    {
        ROS_WARN("Compute path failed!");
        flag=false;
    }
    else
    {
        ROS_WARN("Bad cartesian path! %f", fraction);
        flag=false;
    }
    return flag;
}

bool movePose(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Pose &pose, const std::string &pose_reference_frame)
{
    bool flag=false;
    group.setPoseReferenceFrame(pose_reference_frame);
    ROS_INFO("Start move pose! Reference frame: %s", group.getPoseReferenceFrame().c_str());
    group.setPoseTarget(pose);
    group.setStartStateToCurrentState();
    flag=(group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
    sleep(1);
    if (flag==false)
    {
        ROS_WARN("movePose failed");
    }
    return flag;
}

bool movePoint(moveit::planning_interface::MoveGroupInterface &group, const geometry_msgs::Point &point, const std::string &pose_reference_frame)
{
    bool flag=false;
    group.setPoseReferenceFrame(pose_reference_frame);
    ROS_INFO("Start move point! Reference frame: %s", group.getPoseReferenceFrame().c_str());
    group.setPositionTarget(point.x, point.y, point.z);
    group.setStartStateToCurrentState();
    flag=(group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
    sleep(1);
    if (flag==false)
    {
        ROS_WARN("movePoint failed");
    }
    return flag;
}

bool moveJoints(moveit::planning_interface::MoveGroupInterface &group, const std::vector< double > &joints)
{
    bool flag=false;
    ROS_INFO("Start move joints!");
    group.setJointValueTarget(joints);
    group.setStartStateToCurrentState();
    flag=(group.move()==moveit::planning_interface::MoveItErrorCode::SUCCESS);
    sleep(1);
    if (flag==false)
    {
        ROS_WARN("moveJoints failed");
    }
    return flag;
}

moveit_msgs::CollisionObject createCollisionObject(obj_grasp &og)
{
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = base_link;
    obj.id = og.name;
    if(og.name.size()>=7 && og.name.substr(0,7)=="unknown")
    {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = og.size[0];
        primitive.dimensions[1] = og.size[1];
        primitive.dimensions[2] = og.size[2];
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(og.obj_pose);
    }
    else if(og.name.size()>=5 && og.name.substr(0,5)=="block")
    {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.06;
        primitive.dimensions[1] = 0.06;
        primitive.dimensions[2] = 0.12;
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(og.obj_pose);
    }
    else if(og.name.size()>=8 && og.name.substr(0,8)=="cylinder")
    {
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = 0.13;
        primitive.dimensions[1] = 0.03;
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(og.obj_pose);
    }
    else if(og.name.size()>=7 && og.name.substr(0,7)=="jiantou")
    {
        shapes::Mesh* m = shapes::createMeshFromResource("package://pick_demo_sim/models/jiantou.dae");
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        obj.meshes.push_back(mesh);
        obj.mesh_poses.push_back(og.obj_pose);
    }
    else
    {
        shapes::Mesh* m = shapes::createMeshFromResource("package://pick_demo_sim/models/"+og.name+".dae");
        shapes::ShapeMsg mesh_msg;
        shapes::constructMsgFromShape(m, mesh_msg);
        shape_msgs::Mesh mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        obj.meshes.push_back(mesh);
        obj.mesh_poses.push_back(og.obj_pose);
    }
    obj.operation = obj.ADD;
    return obj;
}

bool next_combination(std::vector<bool> &index)
{
    int n=index.size();
    //从左到右扫描数组
    for(int i=0; i<n-1; i++)
    {
        //找到第一个“10”组合将其变成"01"组合
        if(index[i] && !index[i+1])
        {
            index[i] = false;
            index[i+1] = true;
            //将"01"组合左边的1移到最左边
            int count=0;
            for(int j=0; j<i; j++)
            {
                if(index[j])
                {
                    index[j] = false;
                    index[count++] = true;
                }
            }
            return true;
        }
    }
    return false;
}

bool swap_back(std::vector<int> &vec, int index)
{
    int size=vec.size();
    if(index>=0 && index<=size-2)
    {
        int temp=vec[index];
        vec[index]=vec[index+1];
        vec[index+1]=temp;
        return true;
    }
    return false;
}


