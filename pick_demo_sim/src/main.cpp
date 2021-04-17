#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include "action.h"

using namespace std;
string pkg_path;

void query()
{
    char is_sure;
    std::cout<<"Continue? (y or n) ";
    std::cin>>is_sure;
    if(is_sure=='n')
        exit(1);
}

void input_round(int &rounds)
{
    std::cout<<"round (1-50): ";
    std::cin>>rounds;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pick_demo_sim");
    int round=1;
    if(argc>1)
    {
        stringstream ss;
        ss<<argv[1];
        ss>>round;
    }
    ros::start();
    ros::AsyncSpinner spinner(4);
    spinner.start();

    pkg_path=ros::package::getPath("pick_demo_sim");
    char isplan;
    std::cout<<"Whether grasp planning? (y or n) ";
    std::cin>>isplan;
    Action action;

    if(isplan=='r')
    {
        while(1)
        {
            action.random_arrangement();
            action.file_record();
        }
    }
    if(isplan=='f')
    {
        input_round(action.rounds);
        action.file_arrangement();
        query();
        action.file_record();
    }
    if(isplan=='v')
    {
        while(1)
        {
            input_round(action.rounds);
            action.file_arrangement();
            query();
        }
    }

    if(isplan=='y' || isplan=='n')
    {
        input_round(action.rounds);
//        for(int j=3; j<7; j++)
//        {
//            action.exist_obj_num=j;
//            action.rounds=1;
            for(int i=0; i<round; i++)
            {
                int pick_num=0;
                bool no_plan=false;
                if(isplan=='y')
                {
                    action.initialize();
                    while(action.look_action())
                    {
                        action.grasp_planning();
                        //query();
                        if(!action.pick_object_1(pick_num))
                        {
                            no_plan=true;
                            break;
                        }
                    }
                }
                else if(isplan=='n')
                {
                    action.initialize();
                    while(action.look_action())
                    {
                        if(!action.pick_object_2(pick_num))
                        {
                            no_plan=true;
                            break;
                        }
                        //query();
                    }
                }
                std::cout<<(isplan=='y'?"plan, ":"no plan, ");
                std::cout<<action.exist_obj_num<<" objects, "<<"round "<<action.rounds<<std::endl;
                std::cout<<"total successful grasps: "<<pick_num<<std::endl;
                std::cout<<"total action time: "<<action.action_time<<"s"<<std::endl;
                std::cout<<"total motion planning trials: "<<action.motion_planning_trials<<std::endl;
                std::cout<<"total motion planning time: "<<action.motion_planning_time<<"s"<<std::endl;
                std::cout<<"total obstruction analysis time: "<<action.analysis_time<<"s"<<std::endl;
                std::cout<<"total grasp order planning time: "<<action.planning_time<<"s"<<std::endl;
                std::cout<<"total fallen objects: "<<action.fallen_num<<std::endl;

                stringstream ss;
                ss<<pkg_path<<"/data/results";
//                if(isplan=='y')
//                    ss<<"_plan_";
//                else
//                    ss<<"_noplan_";
//                ss<<action.exist_obj_num;
                ofstream ofs;
                ofs.open(ss.str(), ofstream::app);
                assert(ofs.is_open());
                ofs<<pick_num<<" "<<action.action_time<<" "<<action.motion_planning_trials<<" "<<action.motion_planning_time<<" "
                  <<action.analysis_time<<" "<<action.planning_time<<" "<<action.fallen_num<<" "<<(no_plan?1:0)<<"\n";
                ofs.close();
                action.rounds++;
            }
//        }
    }

    spinner.stop();
    ros::shutdown();
    return 0;

}
