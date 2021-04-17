/*********************************************************************
 视觉抓取项目：抓取桌面上任意摆放的5个物体
 先规划后执行  XY方向推动未知和不可抓取物体
 *********************************************************************/

#include <algorithm>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <random>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetLinkProperties.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <fstream>
#include <chrono>
#include "func.h"
#include "action.h"

using namespace std;
extern string pkg_path;
const int SUCCESS=moveit::planning_interface::MoveItErrorCode::SUCCESS;
const double PI=3.14159265;

void Action::get_current_pose()
{
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform(base_link, tip_link, ros::Time::now(), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }
    current_pose=TransformToPose(transformStamped.transform);
}

Action::~Action()
{
}

Action::Action() : manipulator(arm_group_name), tfListener(tfBuffer)
{
    set_group_config(manipulator);
    ////  use the robot hand
//    hand_pub=node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 1);
//    hand_reset.rACT = 0;
//    hand_active.rACT = 1;
//    hand_active.rGTO = 1;
//    hand_active.rSP  = 70;
//    hand_active.rFR  = 0;
//    hand_close=hand_active;
//    hand_close.rPR = 200;
//    hand_open=hand_active;
//    hand_open.rPR = 70;

    hand_pub=node_handle.advertise<std_msgs::Float64MultiArray>("gripper_controller/command", 1);
    hand_open.data={0.3,-0.3};
    hand_open2.data={0.4,-0.4};
    hand_open3.data={0.5,-0.5};
    hand_open4.data={0.6,-0.6};
    hand_close.data={0.7,-0.7};
    //hand_close.data={0.45,-0.45};

    ////  set many known poses!
    target_joints1[1]=-0.52;
    target_joints1[2]=1.13;
    target_joints1[4]=0.52;

    target_joints2[1]=-1.0;
    target_joints2[2]=1.2;
    target_joints2[4]=-0.2;

    target_joints3[0]=1.5708;
    target_joints3[1]=-0.52;
    target_joints3[2]=1.13;
    target_joints3[4]=0.52;

    target_joints4[0]=1.5708;
    target_joints4[1]=-1.0;
    target_joints4[2]=1.2;
    target_joints4[4]=-0.2;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    arm_model_group = kinematic_model->getJointModelGroup(arm_group_name);
    hand_model_group = kinematic_model->getJointModelGroup(hand_group_name);

    //client = node_handle.serviceClient<pick_msgs::cloud_tran>("get_cloud_position");
    client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    forward.orientation.w=1;
    forward.position.x=0.05;
    backward.orientation.w=1;
    backward.position.x=-0.05;

    double pos_x=-0.15;
    tf2::Quaternion q;
    q.setRPY(0, 0, 1.5708);
    for(int i=1;i<place_0_pose.size();++i)
    {
        place_0_pose[i].position.x=pos_x;
        place_0_pose[i].position.y=0.6;
        pos_x+=0.1;
        place_0_pose[i].orientation=tf2::toMsg(q);
    }
    place_0_pose[0].position.x=0;
    place_0_pose[0].position.y=-0.6;
    place_0_pose[0].position.z=0.09;
    q.setRPY(0, 0, -1.5708);
    place_0_pose[0].orientation=tf2::toMsg(q);

    q.setRPY(0,0,0);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,0.3491);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,-0.3491);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,0.6981);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,-0.6981);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,1.0472);
    place_orientation.push_back(tf2::toMsg(q));
    q.setRPY(0,0,-1.0472);
    place_orientation.push_back(tf2::toMsg(q));

    //set path constraints for the group.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = tip_link;
    ocm.header.frame_id = base_link;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.01;
    ocm.absolute_y_axis_tolerance = 0.01;
    ocm.absolute_z_axis_tolerance = 1.4;
    ocm.weight = 1;
    moveit_msgs::JointConstraint jcm4;
    jcm4.joint_name = "joint_4";
    jcm4.position = 0;
    jcm4.tolerance_below = 1.75;
    jcm4.tolerance_above = 1.75;
    jcm4.weight = 1;
    moveit_msgs::JointConstraint jcm6;
    jcm6.joint_name = "joint_6";
    jcm6.position = 0;
    jcm6.tolerance_below = 1.75;
    jcm6.tolerance_above = 1.75;
    jcm6.weight = 1;
    constraints.orientation_constraints.push_back(ocm);
    //constraints.joint_constraints.push_back(jcm4);
    //constraints.joint_constraints.push_back(jcm6);

    //set ground plane
    ground.header.frame_id = base_link;
    ground.id = "table";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.02;
    ground.primitives.push_back(primitive);
    geometry_msgs::Pose obj_pose;    
    obj_pose.orientation.w=1;
    obj_pose.position.x=0.65;
    obj_pose.position.z=0.01;
    ground.primitive_poses.push_back(obj_pose);
    ground.operation = ground.ADD;

    do {
        cout<<"How many objects existing? (3-6): ";
        cin>>exist_obj_num;
    } while(exist_obj_num<3 || exist_obj_num>6);
}

bool Action::initialize()
{
    name_pose.clear();
    action_time=0;
    planning_time=0;
    motion_planning_trials=0;
    motion_planning_time=0;
    analysis_time=0;
    fallen_num=0;
    inhand_name="";
    find_id.clear();
    discover_id.clear();    
    obj_list_0.clear();
    manipulator.detachObject();
    all_name=planning_scene_interface.getKnownObjectNames();
    planning_scene_interface.removeCollisionObjects(all_name);
    //sleep(1);
//    auto hand_status=ros::topic::waitForMessage<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>("Robotiq2FGripperRobotInput", ros::Duration(1.0));
//    if (hand_status!=NULL && hand_status->gSTA==0)
//    {
//        hand_pub.publish(hand_reset);
//        sleep(1);
//        hand_pub.publish(hand_active);
//        ROS_INFO("Hand Active");
//        sleep(3);
//    }
    success=moveJoints(manipulator, target_joints1);
    hand_pub.publish(hand_open);
    file_arrangement();
    return success;
}

void Action::file_arrangement()
{
    if(rounds==999) return;
    geometry_msgs::Pose pose;
    vector<geometry_msgs::Pose> poses;
    string line, name;
    vector<string> names;
    gazebo_msgs::SetModelState srv;
    stringstream ss;
    ss<<pkg_path<<"/object_arrangement/"<<exist_obj_num<<"/"<<rounds;
    string file_name=ss.str();
    ifstream ifs(file_name);
    assert(ifs.is_open());
    while(getline(ifs,line))
    {
        ss.str("");
        ss.clear();
        ss<<line;
        ss>>name>>pose.position.x>>pose.position.y>>pose.position.z
          >>pose.orientation.x>>pose.orientation.y>>pose.orientation.z>>pose.orientation.w;
        names.push_back(name);
        poses.push_back(pose);
    }
    ifs.close();
    for(int count=0; count<5; count++)
    {
        for(int i=0; i<names.size(); i++)
        {
            srv.request.model_state.model_name=names[i];
            srv.request.model_state.pose=poses[i];
            client.call(srv);
        }
    }
}

void Action::random_arrangement()
{
    char ok='n';
    do{
        gazebo_msgs::SetModelState srv;
        geometry_msgs::Pose pose;
        pose.orientation.w=1;
        for(int i=0; i<6; ++i)
        {
            srv.request.model_state.model_name=id_name.at(i);
            pose.position.x-=1;
            pose.position.z=name_height.at(id_name.at(i))/2;
            srv.request.model_state.pose=pose;
            client.call(srv);
        }
        srv.request.model_state.model_name="board";
        pose.position.x=0.65;
        pose.position.z=0.02;
        srv.request.model_state.pose=pose;
        client.call(srv);
        sleep(1);
        static default_random_engine en(time(0));
        vector<int> ids={0,1,2,3,4,5};
        random_shuffle(ids.begin(), ids.end());
        static uniform_int_distribution<int> random_pose(-12,12);
        for(int i=0; i<exist_obj_num; ++i)
        {
            int id=ids[i];
            srv.request.model_state.model_name=id_name.at(id);
            pose.position.x=random_pose(en)/100.0+0.65;
            pose.position.y=random_pose(en)/100.0;
            pose.position.z=0.3;
            tf2::Quaternion q;
            if(id==5 && random_pose(en)>=0)
            {
                q.setRPY(PI/2,0,random_pose(en)*PI/12);
            }
            else
            {
                q.setRPY(0,0,random_pose(en)*PI/12);
            }
            pose.orientation=tf2::toMsg(q);
            srv.request.model_state.pose=pose;
            client.call(srv);
            sleep(1);
        }
        cout<<"is ok? (y or n or e) ";
        cin>>ok;
        if(ok=='e')
            exit(1);
    }while(ok!='y');
}

void Action::file_record()
{
    cout<<"round: ";
    cin>>rounds;
    stringstream ss;
    ss<<pkg_path<<"/object_arrangement/"<<exist_obj_num<<"/"<<rounds;
    ofstream ofs;
    ofs.open(ss.str());
    assert(ofs.is_open());
    auto model_states=ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
    for(int i=0; i<model_states->name.size(); ++i)
    {
        if(model_states->name[i]=="my_abb" || model_states->name[i]=="ground_plane" || model_states->name[i]=="table" ||
           model_states->name[i]=="board")
        {
            continue;
        }
        ofs<<model_states->name[i]<<" "
           <<model_states->pose[i].position.x<<" "<<model_states->pose[i].position.y<<" "<<model_states->pose[i].position.z<<" "
           <<model_states->pose[i].orientation.x<<" "<<model_states->pose[i].orientation.y<<" "<<model_states->pose[i].orientation.z<<" "
           <<model_states->pose[i].orientation.w<<"\n";
    }
    ofs.close();
}

bool Action::look_action()
{
    planning_scene_interface.removeCollisionObjects(all_name);
    sleep(1);
    int obj_num=0;
    int unknown_num=0;
    obj_list.clear();
    collisions.clear();
    collisions.push_back(ground);
    all_name.clear();
    arrows.clear();
    arrow_name.clear();
    unknown=false;

    //client.waitForExistence();
    //ROS_INFO("service server ready");
    //srv.request.step = 1;
    if(1)
    {
        auto model_states=ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
        //ROS_INFO("find %d objects!",obj_num);
        geometry_msgs::Pose off_table;
        off_table.orientation.w=1;
        for (int i=0; i<model_states->name.size(); ++i)
        {
            if(model_states->name[i]=="my_abb" || model_states->name[i]=="ground_plane" || model_states->name[i]=="table" ||
               model_states->name[i]=="board")
            {
                continue;
            }
            if(model_states->pose[i].position.x<0.5 || model_states->pose[i].position.x>0.8 ||
               model_states->pose[i].position.y<-0.15 || model_states->pose[i].position.y>0.15 ||
               model_states->name[i]==inhand_name)
            {
                gazebo_msgs::SetModelState srv;
                srv.request.model_state.model_name=model_states->name[i];
                off_table.position.x-=1;
                off_table.position.z=name_height.at(model_states->name[i])/2;
                srv.request.model_state.pose=off_table;
                client.call(srv);
                if(name_pose.count(model_states->name[i])==1 && model_states->name[i]!=inhand_name)
                {
                    fallen_num++;
                }
                continue;
            }
            obj_num++;
            obj_grasp og;
            og.name=model_states->name[i];
            og.id=name_id.at(og.name);
            og.obj_pose=model_states->pose[i];
            og.size[0]=0;
            og.size[1]=0;
            og.size[2]=0;
            if(name_pose.count(og.name)==1)
            {
                if(name_pose[og.name].position.z-og.obj_pose.position.z>0.01 ||
                   fabs(name_pose[og.name].position.x-og.obj_pose.position.x)>0.1 ||
                   fabs(name_pose[og.name].position.y-og.obj_pose.position.y)>0.1)
                {
                    fallen_num++;
                }
            }

            if (og.name.size()>=7 && og.name.substr(0,7)=="unknown")
            {
                unknown=true;
                unknown_num++;
                stringstream ss;
                ss<<og.name<<unknown_num;
                og.name=ss.str();
            }
            else
            {
                og.grasp_list.clear();
                XmlRpc::XmlRpcValue my_list;
                ros::param::get("/obj_grasp_pos/"+og.name, my_list);
                ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
                int name_extra=0;
                for(int32_t i = 0; i < my_list.size(); ++i)
                {
                    ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
                    tf2::Quaternion q;
                    tf2::Vector3 v(0, 0, 0.1);
                    q.setRPY(my_list[i][3], my_list[i][4], my_list[i][5]);
                    tf2::Transform transOG(q, tf2::Vector3(my_list[i][0], my_list[i][1], my_list[i][2]));
                    tf2::Transform transBO;
                    tf2::fromMsg(og.obj_pose, transBO);
                    tf2::Transform transBG=transBO*transOG;
                    tf2::Vector3 approach=transBG.getBasis().getColumn(0);
                    tf2::Vector3 upper;
                    if(og.name=="dingshuji")
                    {
                        upper=transBO.getBasis().getColumn(1);
                    }
                    else
                    {
                        upper=transBO.getBasis().getColumn(2);
                    }
                    if(upper.getZ()<0)
                    {
                        upper=-upper;
                    }
                    if(og.name!="mofang" && upper.angle(tf2::Vector3(0,0,1))>0.17)
                    {
                        tf2::Vector3 temp=upper.cross(tf2::Vector3(0,0,1));
                        upper=temp.cross(upper);
                        temp=transBO.getOrigin()-transBG.getOrigin();
                        if(upper.angle(temp)<1.5)
                        {
                            continue;
                        }
                    }
                    double angleF=approach.angle(tf2::Vector3(og.obj_pose.position.x,og.obj_pose.position.y,0));
                    double angleD=approach.angle(tf2::Vector3(0,0,-1));
                    if(angleF<1.6 && angleD<1.6)
                    {
                        grasp_data gd;
                        gd.name=og.name;
                        tf2::toMsg(transBG, gd.grasp_pose);
                        double dist=transBG.getOrigin().distance(v);
                        //dist = max(dist-0.5,0.0);
                        gd.easiness = 1-dist + cos(angleF);
                        //gd.easiness = pow(0.01,dist) + cos(angleF);
                        og.grasp_list.push_back(gd);
//                        q.setRPY(3.14159265, 0, 0);
//                        tf2::Transform transX(q);
//                        transBG*=transX;
//                        tf2::toMsg(transBG, gd.grasp_pose);
//                        gd.difficulty = transBG.getOrigin().distance(v) - fabs(transBG.getRotation().getW());
//                        og.grasp_list.push_back(gd);
                        {
                            obj_grasp jiantou;
                            name_extra++;
                            stringstream ss;
                            ss<<"jiantou_"<<og.name<<name_extra;
                            jiantou.name=ss.str();
                            tf2::toMsg(transBG,jiantou.obj_pose);
                            arrows.push_back(createCollisionObject(jiantou));
                            arrow_name.push_back(jiantou.name);
                        }
                    }
                }
                //sort(og.grasp_list.begin(), og.grasp_list.end(), isEasier1);
            }

            obj_list.push_back(og);
            collisions.push_back(createCollisionObject(og));
            all_name.push_back(og.name);
        }   //end for_cycle
        if(obj_num==0) return false;
        planning_scene_interface.addCollisionObjects(collisions);
        planning_scene_interface.addCollisionObjects(arrows);
        sleep(1);
        sort(obj_list.begin(), obj_list.end());
        name_pose.clear();
        for(const obj_grasp &og : obj_list)
        {
            name_pose[og.name]=og.obj_pose;
        }
        return true;
    }
    else
    {
        ROS_INFO("object recognise failed");
        return false;
    }

}


bool Action::pick_object_1(int &num)
{
    planning_scene_interface.removeCollisionObjects(arrow_name);
    sleep(1);
    string pick_name;
    //do{
        pick_name="";
        for(auto og=obj_list_0.begin(); og!=obj_list_0.end(); og++)
        {
            sort(og->grasp_list.begin(), og->grasp_list.end());
            pick_plan pp(og->id,og->name);
            if(plan_pick(og->grasp_list, pp))
            {
                if(pick_action(pp))
                {
                    num++;
                    pick_name=og->name;
                    obj_list_0.erase(og);
                    return true;//break;
                }
                return false;
            }
        }
        if(pick_name!="")
        {
            for(obj_grasp &og : obj_list_0)
            {
                for(grasp_data &gd : og.grasp_list)
                {
                    gd.total_occ-=gd.occlusion[pick_name];
                }
            }
        }
    //}while(pick_name!="");
    return false;//true;
}

bool Action::pick_object_2(int &num)
{
    planning_scene_interface.removeCollisionObjects(arrow_name);
    sleep(1);
    string pick_name;
    vector<grasp_data> grasp_list;
    while(true)
    {
        grasp_list.clear();
        for(obj_grasp &og : obj_list)
        {
            if(og.name.size()<7 || og.name.substr(0,7)!="unknown")
                grasp_list.insert(grasp_list.end(), og.grasp_list.begin(), og.grasp_list.end());
        }
        if(grasp_list.empty())
        {
            return false;
        }
        sort(grasp_list.begin(), grasp_list.end());
        pick_plan pp(0, "");
        if(plan_pick(grasp_list, pp))
        {
            if(pick_action(pp))
            {
                num++;
                pick_name=pp.name;
                return true;
            }
            else
                return false;
        }
        else
        {
            return false;
        }
        for(auto og=obj_list.begin(); og!=obj_list.end(); og++)
        {
            if(og->name==pick_name)
            {
                obj_list.erase(og);
                break;
            }
        }
    }
}

bool Action::plan_pick(vector<grasp_data> &grasp_list, pick_plan &pp)
{
    bool flag=false;
    auto start=chrono::steady_clock::now();
    kinematic_state->setJointGroupPositions(hand_model_group, open_values);
    for(auto gd=grasp_list.begin(); gd!=grasp_list.end(); ++gd)
    {
        motion_planning_trials++;
        //cout<<motion_planning_trials<<endl;
        if(kinematic_state->setFromIK(arm_model_group, gd->grasp_pose, tip_link, 10, 0.1))
        {
            manipulator.setStartStateToCurrentState();
            manipulator.setJointValueTarget(*kinematic_state);
            if(manipulator.plan(pp.plan1)==SUCCESS)
            {
                if(planPath(manipulator, kinematic_state, forward, tip_link, pp.plan2))
                {
                    pp.name=gd->name;
                    flag=true;
                    break;
                }
            }
        }
    }
    auto end=chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end-start);
    motion_planning_time+=time_span.count();
    return flag;
}

bool Action::pick_action(pick_plan &pp)
{
    auto start=chrono::steady_clock::now();
    manipulator.execute(pp.plan1);
    sleep(1);
    manipulator.execute(pp.plan2);
    sleep(1);
    ROS_INFO("pick %s", pp.name.c_str());
    hand_pub.publish(hand_open2);
    usleep(100000);
    hand_pub.publish(hand_open3);
    usleep(100000);
    hand_pub.publish(hand_open4);
    usleep(100000);
    hand_pub.publish(hand_close);
    manipulator.attachObject(pp.name, tip_link, finger_links);
    sleep(1);

//        ros::ServiceClient client1=node_handle.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
//        gazebo_msgs::GetLinkProperties srv1;
//        srv1.request.link_name=pp.name+"::link";
//        client1.call(srv1);
//        ros::ServiceClient client2=node_handle.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");
//        gazebo_msgs::SetLinkProperties srv2;
//        srv2.request.link_name=pp.name+"::link";
//        srv2.request.gravity_mode=false;
//        srv2.request.com=srv1.response.com;
//        srv2.request.mass=srv1.response.mass;
//        srv2.request.ixx=srv1.response.ixx;
//        srv2.request.iyy=srv1.response.iyy;
//        srv2.request.izz=srv1.response.izz;
//        client2.call(srv2);

    //sleep(1);
    get_current_pose();
    current_pose.position.z+=0.01;
    //current_pose.position.x-=0.01;
    for(int i=0; i<3; i++)
    {
        if(movePath(manipulator, current_pose, base_link, false))
            break;
    }
    //success = moveJoints(manipulator, target_joints2);
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    manipulator.setJointValueTarget(target_joints2);
    manipulator.setStartStateToCurrentState();
    while(manipulator.plan(plan3)!=SUCCESS)// || (plan3.trajectory_.joint_trajectory.points.size()>60))
    {
        get_current_pose();
        current_pose.position.z+=0.01;
        //current_pose.position.x-=0.02;
        movePath(manipulator, current_pose, base_link, false);
        manipulator.setStartStateToCurrentState();
    }
    manipulator.execute(plan3);
    sleep(1);
//    auto hand_status=ros::topic::waitForMessage<robotiq_2f_gripper_control::Robotiq2FGripper_robot_input>("Robotiq2FGripperRobotInput", ros::Duration(1.0));
//    if(hand_status!=NULL && hand_status->gOBJ==3)
//    {
//        hand_pub.publish(hand_open);
//        manipulator.detachObject(pp.name);
//        //sleep(1);
//        planning_scene_interface.removeCollisionObjects(all_name);
//        sleep(1);
//        moveJoints(manipulator, target_joints1);
//        return false;
//    }
    inhand_name=pp.name;
    moveJoints(manipulator, target_joints3);
    hand_pub.publish(hand_open);
    manipulator.detachObject(pp.name);
    vector<string> object_id(1,pp.name);
    planning_scene_interface.removeCollisionObjects(object_id);
    sleep(1);

//        srv2.request.gravity_mode=true;
//        client2.call(srv2);

    //sleep(1);
    moveJoints(manipulator, target_joints1);
    auto end=chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end-start);
    action_time+=time_span.count();
    return true;
}

/*bool Action::plan_place(bool real)
{
    placeable={0,1,1,1,1};
    place_list.clear();
    if(inhand_name=="cylinder0" && !real_plan)
        return true;
//    manipulator.setPathConstraints(constraints);
    robot_state::RobotStatePtr start_state = manipulator.getCurrentState();
    start_state->setJointGroupPositions(hand_model_group, open_values);
    manipulator.setStartState(*start_state);
    manipulator.setPoseReferenceFrame(base_link);
    for(int x=75; x>=55; x-=2)
    {
        if(abs(x-65)<3) continue;
        for(int y=-15; y<=15; y+=2)
        {
            if(abs(y)<3)  continue;
            bool block=false;
            double x1=(double)x/100.0;
            double y1=(double)y/100.0;
            for(obj_grasp &og : obj_list)
            {
                double dx=x1-og.obj_pose.position.x;
                double dy=y1-og.obj_pose.position.y;
                if(sqrt(dx*dx+dy*dy)<0.07 || (dx>0 && fabs(dy)<0.06))
                {
                    block=true;
                    break;
                }
            }
            if(block)  continue;
            place_plan pp(x,y);
            if(!real)
            {
                place_list.push_back(pp);
                if(x<65 && y>0)
                    placeable[1]=0;
                else if(x<65 && y<0)
                    placeable[2]=0;
                else if(x>65 && y>0)
                    placeable[3]=0;
                else if(x>65 && y<0)
                    placeable[4]=0;
                continue;
            }

            geometry_msgs::Pose pos;
            pos.position.x=x1;
            pos.position.y=y1;
            pos.position.z=table_height+0.09;
            for(const geometry_msgs::Quaternion &q : place_orientation)
            {
                pos.orientation=q;
                manipulator.setPoseTarget(pos);
                if(manipulator.plan(pp.plan1)==SUCCESS)
                {
                    place_list.push_back(pp);
                    if(x<65 && y>0)
                        placeable[1]=0;
                    else if(x<65 && y<0)
                        placeable[2]=0;
                    else if(x>65 && y>0)
                        placeable[3]=0;
                    else if(x>65 && y<0)
                        placeable[4]=0;
                    break;
                }
            }

//            manipulator.setPositionTarget(x1,y1,table_height+0.09);
//            bool planning=(manipulator.plan(pp.plan1)==SUCCESS);
//            if(planning && pp.plan1.trajectory_.joint_trajectory.points.size()<20)
//            {
//                place_list.push_back(pp);
//                if(x<65 && y>0)
//                    placeable[1]=0;
//                else if(x<65 && y<0)
//                    placeable[2]=0;
//                else if(x>65 && y>0)
//                    placeable[3]=0;
//                else if(x>65 && y<0)
//                    placeable[4]=0;
//            }

        }
    }
    manipulator.clearPathConstraints();
    return true;
}

bool Action::place_action(int x, int y)
{
    ROS_INFO("place in (%d,%d)", x,y);
    //double start_t=despot::get_time_second();
    success=moveJoints(manipulator, target_joints2);
    if (!success)   return false;
    vector<double> target_joints=target_joints4;
    int i;
    if(x==0 && y==0)
    {
        if(inhand_name=="cylinder0")
        {
            i=0;
            target_joints[0]=-1.5708;
        }
        else
        {
            for(i=1;i<place_0_pose.size();++i)
            {
                if(place_0_pose[i].position.z==0)
                {
                    place_0_pose[i].position.z=0.09;
                    break;
                }
            }
        }
        moveJoints(manipulator, target_joints);
        movePose(manipulator, place_0_pose[i], base_link);
        obj_grasp og;
        og.name=inhand_name;
        og.id=name_id.at(inhand_name);
        og.obj_pose.position.x=place_0_pose[i].position.x;
        og.obj_pose.position.y=place_0_pose[i].position.y;
        og.obj_pose.position.z=0.06;
        og.obj_pose.orientation.w=1;
        og.find=0;
        og.occlusive=0;
        og.graspable=0;
        obj_list_0.push_back(og);
    }
    else
    {
        for(place_plan &pp : place_list)
        {
            if(pp.x==x && pp.y==y)
            {
                if(manipulator.execute(pp.plan1)!=SUCCESS)
                {
                    //cout<<"plan size: "<<pp.plan1.trajectory_.joint_trajectory.points.size()<<". input any letter to continue"<<endl;
                    //char any;
                    //cin>>any;
                    manipulator.stop();
                    sleep(1);
                    std::vector<double> joint_values=pp.plan1.trajectory_.joint_trajectory.points.back().positions;
                    while(!moveJoints(manipulator, joint_values))
                        sleep(1);
                }
            }
        }
    }
    hand_pub.publish(hand_open);
    manipulator.detachObject(inhand_name);
    sleep(1);
    movePath(manipulator, backward, tip_link);
    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    manipulator.setStartStateToCurrentState();
    if(x==0 && y==0)
        manipulator.setJointValueTarget(target_joints);
    else
        manipulator.setJointValueTarget(target_joints2);
    while(manipulator.plan(plan3)!=SUCCESS)// || (plan3.trajectory_.joint_trajectory.points.size()>60))
    {
        get_current_pose();
        //current_pose.position.z+=0.01;
        //current_pose.position.x-=0.02;
        //movePath(manipulator, current_pose, base_link, false);
        manipulator.setStartStateToCurrentState();
    }
    success=(manipulator.execute(plan3)==SUCCESS);
    sleep(1);
    if(x==0 && y==0)
        moveJoints(manipulator, target_joints2);
    //action_time=despot::get_time_second()-start_t;
    all_name.push_back(inhand_name);
    inhand_name="";
    return success;
}

bool Action::plan_place_action(int x, int y)
{
    ROS_INFO("try to plan place in (%d,%d)", x,y);
    success=moveJoints(manipulator, target_joints2);
    if (!success)   return false;
    if(x==0 && y==0)
    {
        return place_action(0,0);
    }
    else
    {
        robot_state::RobotStatePtr start_state = manipulator.getCurrentState();
        start_state->setJointGroupPositions(hand_model_group, open_values);
        manipulator.setStartState(*start_state);
        manipulator.setPoseReferenceFrame(base_link);
        moveit::planning_interface::MoveGroupInterface::Plan plan1;

        geometry_msgs::Pose pos;
        pos.position.x=x/100.0;
        pos.position.y=y/100.0;
        pos.position.z=table_height+0.09;
        for(const geometry_msgs::Quaternion &q : place_orientation)
        {
            pos.orientation=q;
            manipulator.setPoseTarget(pos);
            if(manipulator.plan(plan1)==SUCCESS)
            {
                place_list.clear();
                place_plan pp(x,y);
                pp.plan1=plan1;
                place_list.push_back(pp);
                return place_action(x,y);
            }
        }
        return false;

//        double x1=(double)x/100.0;
//        double y1=(double)y/100.0;
//        manipulator.setPositionTarget(x1,y1,table_height+0.09);
//        manipulator.setPathConstraints(constraints);
//        bool planning=(manipulator.plan(plan1)==SUCCESS);
//        manipulator.clearPathConstraints();
//        if(planning && plan1.trajectory_.joint_trajectory.points.size()<20)
//        {
//            place_list.clear();
//            place_plan pp(x,y);
//            pp.plan1=plan1;
//            place_list.push_back(pp);
//            return place_action(x,y);
//        }
//        else
//            return false;

    }
}

bool Action::plan_push_action(int id)
{
    obj_grasp og;
    for(obj_grasp &copy : obj_list)
    {
        if(copy.id==id)
        {
            og=copy;
            break;
        }
    }
    if(og.name=="")
        return false;
    tf2::Vector3 push_direction;
    geometry_msgs::Pose push_start, push_end, obj_end;
    push_start.orientation.w=1;
    tf2::Transform transBO, transBPO;
    tf2::fromMsg(og.obj_pose, transBO);
    double min_dis=99;
    vector<string> push_name(1);
    for(obj_grasp &og2 : obj_list)
    {
        if(og.name==og2.name || og2.occlusive>0)
            continue;
        double dis=fabs(og.obj_pose.position.y-og2.obj_pose.position.y)-(og.obj_pose.position.x-og2.obj_pose.position.x);
        if(dis<min_dis)
        {
            min_dis=dis;
            tf2::fromMsg(og2.obj_pose, transBPO);
            push_name[0]=og2.name;
        }
    }
    if(min_dis==99)
        return false;
    tf2::Vector3 line(0,1,0);
    if(transBO.getOrigin().getX()>transBPO.getOrigin().getX())
    {
        line=transBPO.getOrigin()-transBO.getOrigin();
        line.setZ(0);
    }
    push_direction=line.cross(tf2::Vector3(0,0,1));
    push_direction.normalize();
    if(push_direction.getX()<0)
        push_direction*= -1;
    cout<<"push direction: ("<<push_direction.getX()<<", "<<push_direction.getY()<<")"<<endl;
    tf2::toMsg(transBPO.getOrigin()-(0.06*push_direction), push_start.position);
    tf2::toMsg(transBPO.getOrigin()+(0.00*push_direction), push_end.position);
    tf2::toMsg(transBPO.getOrigin()+(0.03*push_direction), obj_end.position);
    if(obj_end.position.x>0.75 || fabs(obj_end.position.y)>0.15)
        return false;
    for(obj_grasp &og2 : obj_list)
    {
        if(push_name[0]==og2.name) continue;
        double dx=obj_end.position.x-og2.obj_pose.position.x;
        double dy=obj_end.position.y-og2.obj_pose.position.y;
        if(sqrt(dx*dx+dy*dy)<0.07)
        {
            return false;
        }
    }
    push_start.position.z=push_end.position.z=table_height+0.03;
    bool planning=false;
    push_plan pp(og.id);
    std::vector<double> joint_values;
    kinematic_state->setJointGroupPositions(hand_model_group, close_values);
    for(int i=0; i<3; ++i)
    {
        //manipulator.setGoalOrientationTolerance(1);
        manipulator.setPoseReferenceFrame(base_link);
        kinematic_state->setJointGroupPositions(arm_model_group, target_joints1);
        manipulator.setStartState(*kinematic_state);
        //manipulator.setStartStateToCurrentState();
        manipulator.setPositionTarget(push_start.position.x,push_start.position.y,push_start.position.z);
        if(manipulator.plan(pp.plan1)==SUCCESS)
        {
            if(pp.plan1.trajectory_.joint_trajectory.points.size()>30)
                continue;
            //manipulator.setGoalOrientationTolerance(0.001);
            joint_values=pp.plan1.trajectory_.joint_trajectory.points.back().positions;
            kinematic_state->setJointGroupPositions(arm_model_group, joint_values);
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(tip_link);
            push_start.orientation=push_end.orientation=Eigen::toMsg(end_effector_state).orientation;
            planning_scene_interface.removeCollisionObjects(push_name);
            sleep(1);
            planning=planPath(manipulator, kinematic_state, push_end, base_link, pp.plan2);
            planning_scene_interface.addCollisionObjects(collisions);
            sleep(1);
            if(planning)
            {
                push_list.clear();
                pp.push_start=push_start;
                push_list.push_back(pp);
                return push_action(og.id);
            }
        }
    }
}

bool Action::plan_push()
{
    push_list.clear();
    kinematic_state->setJointGroupPositions(hand_model_group, close_values);
    for(obj_grasp &og : obj_list)
    {
        if(og.occlusive>0 || og.graspable==0)
            continue;

        tf2::Vector3 push_direction;
        //double push_distance;
        geometry_msgs::Pose push_start, push_end, obj_end;
        push_start.orientation.w=1;
        tf2::Transform transBO, transBPO;
        tf2::fromMsg(og.obj_pose, transBO);
        double min_dis=99;
        vector<string> push_name(1);
        for(obj_grasp &og2 : obj_list)
        {
            if(og.name==og2.name || og2.occlusive>0)
                continue;
            double dis=fabs(og.obj_pose.position.y-og2.obj_pose.position.y)-(og.obj_pose.position.x-og2.obj_pose.position.x);
            if(dis<min_dis)
            {
                min_dis=dis;
                tf2::fromMsg(og2.obj_pose, transBPO);
                push_name[0]=og2.name;
            }
        }
        if(min_dis==99)
            continue;
        tf2::Vector3 line(0,1,0);
        if(transBO.getOrigin().getX()>transBPO.getOrigin().getX())
        {
            line=transBPO.getOrigin()-transBO.getOrigin();
            line.setZ(0);
        }
        push_direction=line.cross(tf2::Vector3(0,0,1));
        push_direction.normalize();
        if(push_direction.getX()<0)
            push_direction*= -1;
        cout<<"push direction: ("<<push_direction.getX()<<", "<<push_direction.getY()<<")"<<endl;
        tf2::toMsg(transBPO.getOrigin()-(0.06*push_direction), push_start.position);
        tf2::toMsg(transBPO.getOrigin()+(0.00*push_direction), push_end.position);
        tf2::toMsg(transBPO.getOrigin()+(0.03*push_direction), obj_end.position);
        if(obj_end.position.x>0.75 || fabs(obj_end.position.y)>0.15)
            continue;
        bool block=false;
        for(obj_grasp &og2 : obj_list)
        {
            if(push_name[0]==og2.name) continue;
            double dx=obj_end.position.x-og2.obj_pose.position.x;
            double dy=obj_end.position.y-og2.obj_pose.position.y;
            if(sqrt(dx*dx+dy*dy)<0.07)
            {
                block=true;
                break;
            }
        }
        if(block) continue;
        push_start.position.z=push_end.position.z=table_height+0.03;
        bool planning=false;
        push_plan pp(og.id);
        std::vector<double> joint_values;
        for(int i=0; i<3; ++i)
        {
            //manipulator.setGoalOrientationTolerance(1);
            manipulator.setPoseReferenceFrame(base_link);
            kinematic_state->setJointGroupPositions(arm_model_group, target_joints1);
            manipulator.setStartState(*kinematic_state);
            //manipulator.setStartStateToCurrentState();
            manipulator.setPositionTarget(push_start.position.x,push_start.position.y,push_start.position.z);
            if(manipulator.plan(pp.plan1)==SUCCESS)
            {
                if(pp.plan1.trajectory_.joint_trajectory.points.size()>30)
                    continue;
                //manipulator.setGoalOrientationTolerance(0.001);
                joint_values=pp.plan1.trajectory_.joint_trajectory.points.back().positions;
                kinematic_state->setJointGroupPositions(arm_model_group, joint_values);
                const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(tip_link);
                push_start.orientation=push_end.orientation=Eigen::toMsg(end_effector_state).orientation;
                //planning_scene_interface.removeCollisionObjects(push_name);
                //sleep(1);
                planning=planPath(manipulator, kinematic_state, push_end, base_link, pp.plan2, false);
                //planning_scene_interface.addCollisionObjects(collisions);
                //sleep(1);
                if(planning)
                {
                    pp.push_start=push_start;
                    push_list.push_back(pp);
                    break;
                }
            }
        }
        //manipulator.setGoalOrientationTolerance(0.001);
    }
    return true;
}

bool Action::push_action(int id)
{
    //double start_t=despot::get_time_second();
    for(push_plan &pp : push_list)
    {
        if(pp.id==id)
        {
            hand_pub.publish(hand_close);
            sleep(1);
            manipulator.execute(pp.plan1);
            sleep(1);
            manipulator.execute(pp.plan2);
            sleep(1);
            movePath(manipulator, pp.push_start, base_link, false);
            success=moveJoints(manipulator, target_joints2);
            hand_pub.publish(hand_open);
            sleep(1);
            return success;
        }
    }
    //action_time=despot::get_time_second()-start_t;
    return false;
}
*/


