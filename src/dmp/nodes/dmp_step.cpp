//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include "dmp/DMPPoint.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "tinyxml2.h"
#include "dmp/dmp.h"
#include <fstream>

using namespace std;
using namespace tinyxml2;

bool load_dmp_para(vector<dmp::DMPData>& dmpList,double& tau){
    XMLDocument doc;
    XMLError state = doc.LoadFile("/home/jiyguo/ur_ws/src/dmp/data/dmp2.xml");
    if (state!=tinyxml2::XML_SUCCESS)
    {
        cout << "Can not open this file" << endl;
        return false;
    }
    XMLElement* xml_root = doc.RootElement();
    string dim_str = xml_root->FirstAttribute()->Value();

    dmpList.resize(stoi(dim_str));

    XMLElement* tau_element=xml_root->FirstChildElement("tau");
    string tau_str = tau_element->GetText();
    tau = stof(tau_str);

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
    return true;
}

const int HZ = 100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle n;


    ros::Publisher trajPub = n.advertise<dmp::DMPPointStamp>("dmp/trajectory",100);
    ros::Publisher trajDesPub = n.advertise<dmp::DMPPointStamp>("dmp/des_trajectory",100);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory",10, true);
    ros::Publisher pathDesPub = n.advertise<nav_msgs::Path>("xy/des_trajectory",10, true);

    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    nav_msgs::Path pathDes;
    pathDes.header.frame_id="world";


    vector<dmp::DMPData> dmpList;
    double tau;
    if(!load_dmp_para(dmpList, tau)){
        return -1;
    }

    std::vector<double> x_0; x_0.push_back(0.22); x_0.push_back(-0.22);
    std::vector<double> x_dot_0; x_dot_0.push_back(0.0); x_dot_0.push_back(0.0);
    std::vector<double> goal; goal.push_back(0.14); goal.push_back(-0.38);
    std::vector<double> goal_thresh; goal_thresh.push_back(0.01); goal_thresh.push_back(0.01);
    
    double seg_length = 20;
    double total_dt = 1.0 / HZ;
    int integrate_iter = 1;
    double cur_t = 0;

    dmp::DMPPoint cur_state; cur_state.positions = x_0; cur_state.velocities = x_dot_0;
    dmp::DMPPoint plan;
    bool at_goal = false;

    int dims = x_0.size();
    dmp::FunctionApprox **f = new dmp::FunctionApprox*[dims];
    for(int i=0; i<dims; i++){
        double base_width = pow(dmpList[i].weights.size(),1.5) * dmp::alpha;
        f[i] = new dmp::RadialApprox(dmpList[i].weights, base_width, dmp::alpha);
    }

    ros::Rate rate(HZ);
    dmp::DMPPointStamp cur;
    while(ros::ok() &!at_goal && cur_t < seg_length) {
        generateStep(dmpList, x_0, cur_t, goal, goal_thresh, tau, total_dt,
                     integrate_iter, f, cur_state, plan, at_goal);

        path.header.stamp = ros::Time::now();
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.position.x = plan.positions[0];
        this_pose_stamped.pose.position.y = plan.positions[1];
        this_pose_stamped.pose.position.z = 0;
        path.poses.push_back(this_pose_stamped);
        pathPub.publish(path);

        // update state
        cur_state = plan;
        cur_t += total_dt;
        rate.sleep();
    }

    //Clean up
    for(int i=0; i<dims; i++){
        delete f[i];
    }
    delete[] f;

    return 0;
}