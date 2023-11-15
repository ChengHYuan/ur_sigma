//
// Created by jiy on 2021/11/23.
//

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

nav_msgs::Path points;
int PathGen(vector<vector<double>>& path,double& length,
            const string& xml_path, const dmp::GetDMPPlan& dmpPlan,
            const double& max_len, ros::NodeHandle &n)
{
    dmp::GetDMPPlan srvPlan;
    srvPlan.request = dmpPlan.request;
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory4",10, true);
    points.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";
    ros::Rate rate(10);

    length = 0.0;
    path.clear();
    path.resize(2);
    ros::ServiceClient clientPlan= n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
    XMLDocument doc;
    XMLError state = doc.LoadFile(xml_path.c_str());
    if (state!=tinyxml2::XML_SUCCESS)
    {
        cout << "Can not open this file" << endl;
        return 0;
    }
    XMLElement* xml_root = doc.RootElement();
    string dim_str = xml_root->FirstAttribute()->Value();

    vector<dmp::DMPData> dmpList(stoi(dim_str));

    XMLElement* tau_element=xml_root->FirstChildElement("tau");
    string tau_str = tau_element->GetText();
//    float tau = stof(tau_str);

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

    double pre_x = srvPlan.request.x_0[0];
    double pre_y = srvPlan.request.x_0[1];

    if(clientActive.call(srvSetActive)){
        if(clientPlan.call(srvPlan) && srvPlan.response.at_goal){
//            ROS_INFO("DMP services call successful ! ");
            traj = srvPlan.response.plan;
            for (auto& point:traj.points) {
                if(length>max_len) break;
                path[0].push_back(point.positions[0]);
                path[1].push_back(point.positions[1]);
                length += std::sqrt(pow(point.positions[0] - pre_x,2) + pow(point.positions[1] - pre_y,2));
                pre_x = point.positions[0];
                pre_y = point.positions[1];
/*
 *
 *   visulization
 *
 *              points.header.stamp = ros::Time::now();
                this_pose_stamped.header.stamp = ros::Time::now();
                this_pose_stamped.pose.position.x = point.positions[0];
                this_pose_stamped.pose.position.y = point.positions[1];
                this_pose_stamped.pose.position.z = 0;
                points.poses.push_back(this_pose_stamped);
                pathPub.publish(points);
                rate.sleep();
 *
 * */
            }
            return 1;
        }else{
            ROS_INFO("clientPlan services call failed ! ");
            return 0;
        }
    }else {
        ROS_INFO("Failled call DMP services! ");
        return 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_test");
    ros::NodeHandle n;
    ros::Publisher probPub = n.advertise<geometry_msgs::PointStamped>("dmp/probability",10, true);

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
    vector<vector<double>> path;
    vector<vector<vector<double>>> dmp_paths;

    dmp::GetDMPPlan srvPlan;
    srvPlan.request.x_0.push_back(X0);srvPlan.request.x_0.push_back(Y0);
    srvPlan.request.x_dot_0.push_back(0);srvPlan.request.x_dot_0.push_back(0);
    srvPlan.request.t_0 = 0;
    srvPlan.request.goal.push_back(XG);srvPlan.request.goal.push_back(YG);
    srvPlan.request.goal_thresh.push_back(0.002);srvPlan.request.goal_thresh.push_back(0.002);
    srvPlan.request.seg_length = 20;
    srvPlan.request.tau = 5.0;
    srvPlan.request.dt = 0.008;
    srvPlan.request.integrate_iter = 1;
    double length = 0.0;

    geometry_msgs::PointStamped prob;

    double cur_max_len = 0;
    double step_max_len = 0.0005;
    double max_max_len = 0.35;
    xml_name = xml_path + "dmp23.xml";
    ros::Rate rate(100);
    vector<double> confs;


    clock_t start,end;

    while(cur_max_len<max_max_len) {
        start=clock();	
        cur_max_len+=step_max_len;
        dmp_paths.clear();
        for(string& xml:xml_names){
            if(PathGen(path,length,xml,srvPlan,cur_max_len,n)){
                dmp_paths.push_back(path);
                path.clear();
//                cout<<"Length: "<<length<<endl;
            }
        }
        PathGen(path,length,xml_name,srvPlan,cur_max_len,n);
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