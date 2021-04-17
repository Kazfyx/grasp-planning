#include "action.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Aff_transformation_3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

using namespace std;
extern string pkg_path;

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Vector_3 Vector;
typedef K::Ray_3 Ray;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef Polyhedron::Vertex_iterator Vertex_iterator;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional<Tree::Intersection_and_primitive_id<Ray>::Type> Ray_intersection;
typedef Tree::Primitive_id Primitive_id;
const double PI=3.14159265;

struct Node
{
    std::vector<std::string> grasp_order;
    double lowerbound=0, cost=0;
    std::map<std::string, std::map<std::string, double>> curr_obj_occ;
    bool operator<(const Node &node) const
    {
        return lowerbound > node.lowerbound;
    }
};

bool Action::grasp_planning()
{
    //build CGAL object model
    std::map<std::string, Polyhedron> object_model;
    for(obj_grasp &og : obj_list)
    {
        Polyhedron P;
        if(og.name.size()>=7 && og.name.substr(0,7)=="unknown")
        {
            std::ifstream ifs(pkg_path+"/models/mofang.off");
            assert(ifs.is_open());
            ifs>>P;
            ifs.close();
            for(Vertex_iterator v=P.vertices_begin(); v!=P.vertices_end(); ++v)
            {
                double x=v->point().x()>0 ? og.size[0]/2 : -og.size[0]/2;
                double y=v->point().y()>0 ? og.size[1]/2 : -og.size[1]/2;
                double z=v->point().z()>0 ? og.size[2]/2 : -og.size[2]/2;
                v->point()=Point(x,y,z);
            }
            P.is_valid();
        }
        else
        {
            std::ifstream ifs(pkg_path+"/models/"+og.name+".off");
            assert(ifs.is_open());
            ifs>>P;
            ifs.close();
        }
        tf2::Quaternion q;
        tf2::fromMsg(og.obj_pose.orientation, q);
        tf2::Matrix3x3 m(q);
        CGAL::Aff_transformation_3<K> A(m.getRow(0).getX(),m.getRow(0).getY(),m.getRow(0).getZ(),og.obj_pose.position.x,
                                        m.getRow(1).getX(),m.getRow(1).getY(),m.getRow(1).getZ(),og.obj_pose.position.y,
                                        m.getRow(2).getX(),m.getRow(2).getY(),m.getRow(2).getZ(),og.obj_pose.position.z,1.0);
        std::transform(P.points_begin(), P.points_end(), P.points_begin(), A);
        object_model[og.name]=P;
    }

    auto start1=chrono::steady_clock::now();
    //occlusion analysis
    for(auto &om : object_model)
    {
        Tree tree(faces(om.second).first, faces(om.second).second, om.second);
        for(obj_grasp &og : obj_list)
        {
            if(og.name.size()>=7 && og.name.substr(0,7)=="unknown")
                continue;
            //og.occlusion[om.first]=0;
            if(om.first==og.name)
                continue;
            double total_easiness=0, total_occlusion=0;
            for(grasp_data &gd : og.grasp_list)
            {
                total_easiness+=1;//gd.easiness;
                tf2::Transform transBG;
                tf2::fromMsg(gd.grasp_pose, transBG);
                tf2::Vector3 x_axis=transBG.getBasis().getColumn(0);
                tf2::Vector3 y_axis=transBG.getBasis().getColumn(1);
                tf2::Vector3 cp=transBG.getOrigin()+0.03*x_axis;
                Point p(cp.getX(),cp.getY(),cp.getZ());
                Vector v(-x_axis.getX(),-x_axis.getY(),-x_axis.getZ());
                Ray r(p,v);
                if(tree.number_of_intersected_primitives(r)==1)
                {
                    total_occlusion+=1;//gd.easiness;
                    gd.occlusion[om.first]=1;
                    gd.total_occ+=1;
                    continue;
                }
                std::list<Ray_intersection> intersections;
                double min_dis=100;
                for(int i=0; PI/18*i<=PI/9+1e-6; i++)
                {
                    tf2::Vector3 v0=(-x_axis).rotate(y_axis,PI/18*i);
                    for(int j=0; PI/18*j<=PI*2+1e-6; j++)
                    {
                        tf2::Vector3 v1=v0.rotate(-x_axis,PI/18*j);
                        Vector vec(v1.getX(),v1.getY(),v1.getZ());
                        Ray ray(p,vec);
                        intersections.clear();
                        tree.all_intersections(ray, std::back_inserter(intersections));
                        for(Ray_intersection &intersect : intersections)
                        {
                            Point* q=boost::get<Point>(&(intersect->first));
                            if(q)
                            {
                                double dis=sqrt(CGAL::squared_distance(p,*q));
                                if(dis<min_dis)
                                    min_dis=dis;
                            }
                        }
                        if(i==0) break;
                    }
                }
                if(min_dis<10)
                {
                    total_occlusion+=pow(0.01,min_dis);//*gd.easiness;
                    gd.occlusion[om.first]=pow(0.01,min_dis);
                    gd.total_occ+=pow(0.01,min_dis);
                }
                else
                {
                    gd.occlusion[om.first]=0;
                }
            }
            if(om.first.size()>=7 && om.first.substr(0,7)=="unknown")
                og.occlusion["unknown"]+=total_occlusion/total_easiness;
            else
                og.occlusion[om.first]=total_occlusion/total_easiness;
        }
    }
    auto end1=chrono::steady_clock::now();
    chrono::duration<double> time_span1 = chrono::duration_cast<chrono::duration<double>>(end1-start1);
    analysis_time+=time_span1.count();

    //prepare data for planning
    std::vector<std::string> known_obj;
    std::map<std::string, std::map<std::string, double>> obj_occlusion;
    for(auto &og : obj_list)
    {
        if(og.name.size()<7 || og.name.substr(0,7)!="unknown")
        {
            std::cout<<og.name<<": ";
            known_obj.push_back(og.name);
            obj_occlusion[og.name]=og.occlusion;
            double occ=0;
            for(auto &oc : og.occlusion)
            {
                std::cout<<oc.first<<"["<<oc.second<<"] ";
                occ+=oc.second;
            }
            std::cout<<"total["<<occ<<"]"<<std::endl;
        }
    }

    auto start=chrono::steady_clock::now();
    //clock_t start=clock();
    //compute initial upper bound by greed method
    std::vector<std::string> optimal_grasp_order;
    double upperbound=0;
    auto curr_obj_occ=obj_occlusion;
    while(!curr_obj_occ.empty())
    {
        std::string min_obj;
        double min_occ=10000;
        for(auto &obj : known_obj)
        {
            if(find(optimal_grasp_order.begin(),optimal_grasp_order.end(),obj)!=optimal_grasp_order.end())
                continue;
            double occ=0;
            for(auto &oc : curr_obj_occ[obj])
            {
                occ+=oc.second;
            }
            if(occ<min_occ)
            {
                min_obj=obj;
                min_occ=occ;
            }
        }
        optimal_grasp_order.push_back(min_obj);
        upperbound+=min_occ;
        curr_obj_occ.erase(min_obj);
        for(auto &ooc : curr_obj_occ)
        {
            ooc.second.erase(min_obj);
        }
    }

    //branch and bound method
    priority_queue<Node> Q;
    Node node0;
    node0.curr_obj_occ=obj_occlusion;
    Q.push(node0);
    while(!Q.empty())
    {
        Node node=Q.top();
        Q.pop();
        if(node.lowerbound>upperbound)
            break;
        for(auto &obj : known_obj)
        {
            if(find(node.grasp_order.begin(),node.grasp_order.end(),obj)!=node.grasp_order.end())
                continue;
            Node new_node=node;
            new_node.grasp_order.push_back(obj);
            double occ=0;
            for(auto &oc : new_node.curr_obj_occ[obj])
            {
                occ+=oc.second;
            }
            new_node.cost+=occ;
            new_node.curr_obj_occ.erase(obj);
            for(auto &ooc : new_node.curr_obj_occ)
            {
                ooc.second.erase(obj);
            }

            if(new_node.curr_obj_occ.empty())
            {
                if(new_node.cost<upperbound)
                {
                    upperbound=new_node.cost;
                    optimal_grasp_order=new_node.grasp_order;
                }
            }
            else
            {
                new_node.lowerbound=new_node.cost;
                double min_occ=10000;
                for(auto &ooc : new_node.curr_obj_occ)
                {
                    new_node.lowerbound+=ooc.second["unknown"];
                    double occ=0;
                    for(auto &oc : ooc.second)
                    {
                        if(oc.first!="unknown")
                            occ+=oc.second;
                    }
                    if(occ<min_occ)
                    {
                        min_occ=occ;
                    }
                }
                new_node.lowerbound+=min_occ;
                if(new_node.lowerbound<upperbound)
                    Q.push(new_node);
            }
        }
    }
    auto end=chrono::steady_clock::now();
    chrono::duration<double> time_span = chrono::duration_cast<chrono::duration<double>>(end-start);
    planning_time+=time_span.count();
    //clock_t end=clock();
    //planning_time+=double(end-start)/CLOCKS_PER_SEC;

    //sort obj_list_0
    obj_list_0.clear();
    std::cout<<"optimal grasp order:\n";
    for(std::string &name : optimal_grasp_order)
    {
        std::cout<<name<<std::endl;
        for(obj_grasp &og : obj_list)
        {
            if(og.name==name)
            {
                obj_list_0.push_back(og);
                break;
            }
        }
    }

    return true;
}
