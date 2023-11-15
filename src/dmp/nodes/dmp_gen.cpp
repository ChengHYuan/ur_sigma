//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "tinyxml2.h"

#include <fstream>

using namespace std;
using namespace tinyxml2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle n;

    ros::ServiceClient clientPlan= n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    ros::Publisher trajPub = n.advertise<dmp::DMPPointStamp>("dmp/trajectory",100);
    ros::Publisher trajDesPub = n.advertise<dmp::DMPPointStamp>("dmp/des_trajectory",100);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory2",10, true);
    ros::Publisher pathDesPub = n.advertise<nav_msgs::Path>("xy/des_trajectory",10, true);

    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    nav_msgs::Path pathDes;
    pathDes.header.frame_id="world";

    XMLDocument doc;
    XMLError state = doc.LoadFile("/home/jiyguo/ur_ws/src/dmp/data/dmp2.xml");
    if (state!=tinyxml2::XML_SUCCESS)
    {
        cout << "Can not open this file" << endl;
        return -1;
    }
    XMLElement* xml_root = doc.RootElement();
    string dim_str = xml_root->FirstAttribute()->Value();

    vector<dmp::DMPData> dmpList(stoi(dim_str));

    XMLElement* tau_element=xml_root->FirstChildElement("tau");
    string tau_str = tau_element->GetText();
    float tau = stof(tau_str);

    XMLElement *dmp_element = xml_root->FirstChildElement("dmp");
    int idx = 0;
    while (dmp_element)
    {
        XMLElement *kgain_element = dmp_element->FirstChildElement("kgain");
        string kgain_str = kgain_element->GetText();
        dmpList[idx].k_gain = stof(kgain_str);
        XMLElement *dgain_element = dmp_element->FirstChildElement("dgain");
        string dgain_str = dgain_element->GetText();
        dmpList[idx].d_gain = stof(dgain_str);
        XMLElement *weights_element = dmp_element->FirstChildElement("weights");
        string nums_weight_str = weights_element->FirstAttribute()->Value();
        vector<double> weights(stoi(nums_weight_str));
        XMLElement *w_element = weights_element->FirstChildElement("w");
        for (int i = 0; i < weights.size(); ++i) {
            string w = w_element->GetText();
            weights[i] = stof(w);
            w_element=w_element->NextSiblingElement();
        }
        dmpList[idx].weights = weights;
        dmp_element = dmp_element->NextSiblingElement();
        idx++;
    }

    dmp::SetActiveDMP srvSetActive;
    dmp::DMPTraj traj;
    srvSetActive.request.dmp_list = dmpList;
    if(clientActive.call(srvSetActive)){
        dmp::GetDMPPlan srvPlan;
//        srvPlan.request.x_0.push_back(0.16);srvPlan.request.x_0.push_back(-0.15);
//        srvPlan.request.x_0.push_back(0.22);srvPlan.request.x_0.push_back(-0.2);
        srvPlan.request.x_0.push_back(0.22);srvPlan.request.x_0.push_back(-0.22);

        srvPlan.request.x_dot_0.push_back(0);srvPlan.request.x_dot_0.push_back(0);
        srvPlan.request.t_0 = 0;

//        srvPlan.request.goal.push_back(0.15);srvPlan.request.goal.push_back(-0.33);
//        srvPlan.request.goal.push_back(0.1);srvPlan.request.goal.push_back(-0.37);
        srvPlan.request.goal.push_back(0.14);srvPlan.request.goal.push_back(-0.38);

        srvPlan.request.goal_thresh.push_back(0.01);srvPlan.request.goal_thresh.push_back(0.01);
        srvPlan.request.seg_length = 20;
        srvPlan.request.tau = tau;
        srvPlan.request.dt = 0.01;
        srvPlan.request.integrate_iter = 1;
        if(clientPlan.call(srvPlan) && srvPlan.response.at_goal){
            traj = srvPlan.response.plan;
        }else{
            ROS_INFO("clientPlan services call failed ! ");
        }
    }
    ROS_INFO("DMP services call successful ! ");
    dmp::DMPPointStamp cur;
    dmp::DMPPointStamp des;
    ros::Rate rate(100);
    for (int i = 0; i < traj.points.size(); ++i) {
        cur.head.stamp = ros::Time::now();
        cur.positions = traj.points[i].positions[0];
        trajPub.publish(cur);

        double time = i * 0.01;
        des.head.stamp = ros::Time::now();
        des.positions = cos(0.2 * M_PI * time);
        trajDesPub.publish(des);

        path.header.stamp = ros::Time::now();
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.position.x = traj.points[i].positions[0];
        this_pose_stamped.pose.position.y = traj.points[i].positions[1];
        this_pose_stamped.pose.position.z = 0;
        path.poses.push_back(this_pose_stamped);
        pathPub.publish(path);

        rate.sleep();
    }

    return 0;
}