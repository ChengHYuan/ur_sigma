
#include "confidence.h"

//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <fstream>

#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

const int NUMBASES = 15;
const double X0 = 0.16264;
const double Y0 = -0.1453;

const double XG = 0.14541;
const double YG = -0.34241;


void SrvInit(dmp::GetDMPStepPlan& srvPlan){
    srvPlan.request.x_0.push_back(X0);srvPlan.request.x_0.push_back(Y0);
    srvPlan.request.t_0 = 0;
    srvPlan.request.goal.push_back(XG);srvPlan.request.goal.push_back(YG);
    srvPlan.request.goal_thresh.push_back(0.002);srvPlan.request.goal_thresh.push_back(0.002);
    srvPlan.request.seg_length = 20;
    srvPlan.request.tau = 5.0;
    srvPlan.request.dt = 0.008;
    srvPlan.request.integrate_iter = 1;
    srvPlan.request.current.positions = srvPlan.request.x_0;
    srvPlan.request.current.velocities.push_back(0);srvPlan.request.current.velocities.push_back(0);
}

nav_msgs::Path points;
int PathGen(dmp::FunctionApprox **f,vector<dmp::DMPPoint>& path , double& length,double& time,
            const vector<dmp::DMPData>& dmpList, dmp::GetDMPStepPlan& dmpStepPlan,
            const double& max_len, ros::NodeHandle &n)
{

    /*
     *
     * visualizes
     *
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory4",10, true);
    points.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";
     */

//    ros::Rate rate(10);

//    length = 0.0;
//    dmp::GetDMPStepPlan dmpStepPlan = tmpSrv;

    // find pre point
    vector<double> pre_pos(2);
//    cout<<path.size()<<endl;
    if(path.empty()){
        pre_pos[0] = dmpStepPlan.request.x_0[0];
        pre_pos[1] = dmpStepPlan.request.x_0[1];
        dmpStepPlan.request.current.positions = dmpStepPlan.request.x_0;
//        dmpStepPlan.request.current.velocities.push_back(0);dmpStepPlan.request.current.velocities.push_back(0);
    } else{
        pre_pos[0] = path.back().positions[0];
        pre_pos[1] = path.back().positions[1];
        dmpStepPlan.request.current.positions = path.back().positions;
        dmpStepPlan.request.current.velocities = path.back().velocities;
    }
    double x,y;
    bool atGoal= false;
    dmp::DMPPoint plan;
    while (length<=max_len && !atGoal) {
        dmpStepPlan.request.t_0 = time;
        generateStep(dmpList, dmpStepPlan.request.x_0, dmpStepPlan.request.t_0, dmpStepPlan.request.goal, dmpStepPlan.request.goal_thresh, dmpStepPlan.request.tau, dmpStepPlan.request.dt,
                     dmpStepPlan.request.integrate_iter, f, dmpStepPlan.request.current, plan, atGoal);


        x = plan.positions[0];
        y = plan.positions[1];
        length += std::sqrt(pow(x - pre_pos[0],2) + pow(y - pre_pos[1],2));
        path.push_back(plan);
        dmpStepPlan.request.current = plan;
        pre_pos[0] = x;
        pre_pos[1] = y;
        time+=dmpStepPlan.request.dt;



/*
*
*   visulization
*
*
*
        points.header.stamp = ros::Time::now();
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.position.x = plan.positions[0];
        this_pose_stamped.pose.position.y = plan.positions[1];
        this_pose_stamped.pose.position.z = 0;
        points.poses.push_back(this_pose_stamped);
        pathPub.publish(points);
        rate.sleep();
*
* */

    }
    return 1;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "dmp_test");
    ros::NodeHandle n;
    ros::Publisher probPub = n.advertise<geometry_msgs::PointStamped>("dmp/probability",10, true);
    ros::Rate rate(100);

    ofstream my_data("/home/jiyguo/ur_ws/src/dmp/data/prob2.txt",ios::out);

    string xml_path = "/home/jiyguo/ur_ws/src/dmp/data/";
    string xml_name;
    vector<string> xml_names;
    xml_name = xml_path + "dmp01.xml";
    xml_names.push_back(xml_name);
    xml_name = xml_path + "dmp11.xml";
    xml_names.push_back(xml_name);
    xml_name = xml_path + "dmp21.xml";
    xml_names.push_back(xml_name);

    vector<vector<dmp::DMPData>> dmpLists(3);
    vector<double> taus(3,0);
    double tau_demo=0;
    vector<double> times(3,0);
    double time_demo=0;
    for (int i=0;i<xml_names.size();++i) {
        string xml = xml_names[i];
        load_dmp_para( xml, dmpLists[i], taus[i]);
    }
    size_t dims = dmpLists[0].size();
    // construct the force term function by dmp parameters

//    vector<vector<double>> path;
//    vector<vector<vector<double>>> dmp_paths;
    vector<dmp::GetDMPStepPlan> srvPlan(3);
    dmp::GetDMPStepPlan srvPlan_demo;
    SrvInit(srvPlan_demo);
    for (int i = 0; i < srvPlan.size(); ++i) {
        SrvInit(srvPlan[i]);
    }
    vector<double> length(3,0);
    double length_demo = 0;

    geometry_msgs::PointStamped prob;

    double cur_max_len = 0;
    double step_max_len = 0.001;
    double max_max_len = 0.35;
    xml_name = xml_path + "dmp03.xml";
    vector<dmp::DMPData> dmpList_demo;
    load_dmp_para( xml_name, dmpList_demo, tau_demo);

    vector<double> confs;
    clock_t start,end;
    vector<vector<dmp::DMPPoint>> dmp_paths(3);
    vector<dmp::DMPPoint> path;
    vector<dmp::FunctionApprox **> f_dmp(3);
    for (int i = 0; i < 3; ++i) {
        f_dmp[i] = new dmp::FunctionApprox*[dims];
        for(int j=0; j<dims; j++){
            double base_width = pow(dmpLists[i][j].weights.size(),1.5) * dmp::alpha;
            f_dmp[i][j] = new dmp::RadialApprox(dmpLists[i][j].weights, base_width, dmp::alpha);
        }
    }
    dmp::FunctionApprox **f_demo;
    f_demo = new dmp::FunctionApprox*[dims];
    for(int j=0; j<dims; j++){
        double base_width = pow(dmpList_demo[j].weights.size(),1.5) * dmp::alpha;
        f_demo[j] = new dmp::RadialApprox(dmpList_demo[j].weights, base_width, dmp::alpha);
    }

    while(cur_max_len<max_max_len) {
        start=clock();
        cur_max_len+=step_max_len;
        for(int i=0;i<xml_names.size();++i){
            srvPlan[i].request.tau = taus[i];
            PathGen(f_dmp[i],dmp_paths[i],length[i],times[i],dmpLists[i],srvPlan[i],cur_max_len,n);
//                cout<<"Length: "<<length<<endl;

        }
        PathGen(f_demo,path,length_demo,time_demo,dmpList_demo,srvPlan_demo,cur_max_len,n);
//        cout<<"cur_max_len: "<<length<<endl;
//        confs.clear();
        confs = Confidances(dmp_paths, path);
//        cout<<"confs.size(): "<<confs.size()<<endl;
//        for_each(confs.begin(),confs.end(),[](double& it){std::cout<<it<<endl;});
        end=clock();
        double endtime=(double)(end-start)/CLOCKS_PER_SEC;
        string info  = "Total time:" + to_string(endtime*1000) + "ms";
        ROS_INFO("%s", info.c_str());
        // ROS_INFO("Total time: %d ms",endtime*1000);
        // cout<<"Total time:"<<endtime*1000<<"ms"<<endl;	//ms为单位
        prob.header.stamp = ros::Time::now();
        prob.point.x =confs[0];
        prob.point.y =confs[1];
        prob.point.z =confs[2];
        my_data<<cur_max_len<<" "<<prob.point.x<<" "<<prob.point.y<<" "<<prob.point.z<<endl;
        probPub.publish(prob);
        rate.sleep();
    }
    my_data.close();
//    for_each(confs.begin(),confs.end(),[](double& it){std::cout<<it<<endl;});
    return 0;
}