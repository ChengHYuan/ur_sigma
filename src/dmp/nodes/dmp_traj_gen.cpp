#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include "dmp/DMPPoint.h"
#include "dmp/GoalToPath.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Float32MultiArray.h"
#include "tinyxml2.h"
#include "dmp/dmp.h"
#include <fstream>
#include <Eigen/Dense>

using namespace std;
using namespace tinyxml2;

geometry_msgs::PoseStamped origin;
vector<vector<double>>goals = { {0.6645, -0.0375}, {0.6645, -0.0125}, {0.6645, 0.0125}, {0.6645, 0.0375}, {0.6895, -0.0125}, {
        0.6895, 0.0125}, {0.7145, -0.0125}, {0.7145, 0.0125}, {0.7395, -0.0375}, {0.7395, -0.0125}, {0.7395, 0.0125}, {0.7395, 0.0375} };

vector<double> goal(2);

int my_index = -1;
int my_index_temp = -1;
vector<double> my_start_point;


void goal_callback(const std_msgs::Float32MultiArray& msg) {
    my_index_temp = msg.data[0];
}

bool goal_server_callback(dmp::GoalToPath::Request& req,dmp::GoalToPath::Response &res) {
    my_start_point = req.Start;
    my_index_temp = int(req.GoalIndex);
    
}

bool load_dmp_para(vector<dmp::DMPData>& dmpList, double& tau) {
    XMLDocument doc;
    XMLError state = doc.LoadFile("/home/chy/sigma_iiwa_simulation/sigma_to_sim_workspace/myXML.xml");
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
    // cout << "flag1" << endl;
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

    // cout << "dmplist weight size :"<<dmpList[0].weights.size() << endl;
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "dmp_traj_gen");
    ros::NodeHandle n;
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
    ros::ServiceClient clientPlan = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("/trajectory", 10, true);
    // ros::Publisher iiwa_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose_origin", 1, true);
    ros::Subscriber goal_sub = n.subscribe("/goal", 1, goal_callback);
    // ros::ServiceServer goal_server = n.advertiseService("/goal", goal_server_callback);

    vector<dmp::DMPData> dmpList;
    dmp::DMPTraj traj;
    nav_msgs::Path path;
    path.header.frame_id = "world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";
    
    double tau;
    
    if (!load_dmp_para(dmpList, tau)) {
        return -1;
    }
    
    dmp::SetActiveDMP srvSetActive;
    ros::Rate rate(20);
    srvSetActive.request.dmp_list = dmpList;
    ROS_INFO("-------waiting for new goal---------");
    while (ros::ok()) {
        if (my_index != my_index_temp) {
            ROS_INFO("get a new goal!!");
            goal = goals[my_index_temp];
            if (clientActive.call(srvSetActive)) {
                dmp::GetDMPPlan srvPlan;
                // srvPlan.request.x_0.push_back(points[0].positions[0]);srvPlan.request.x_0.push_back(points[0].positions[1]);//设置起始位置
                srvPlan.request.x_0.push_back(0.6645);srvPlan.request.x_0.push_back(0.0375);//设置起始位置
                srvPlan.request.x_dot_0.push_back(0);srvPlan.request.x_dot_0.push_back(0);
                srvPlan.request.t_0 = 0;
                // srvPlan.request.goal.push_back(points[points.size() - 1].positions[0]);srvPlan.request.goal.push_back(points[points.size() - 1].positions[1]);//设置目标位置
                srvPlan.request.goal.push_back(goal[11]);srvPlan.request.goal.push_back(goal[11]);
                srvPlan.request.goal_thresh.push_back(0.001);srvPlan.request.goal_thresh.push_back(0.001);
                // srvPlan.request.seg_length = 20;
                srvPlan.request.seg_length = -1;
                srvPlan.request.tau = tau;
                srvPlan.request.dt = 0.01;
                srvPlan.request.integrate_iter = 1;
                if (clientPlan.call(srvPlan) && srvPlan.response.at_goal) {
                    traj = srvPlan.response.plan;
                    ROS_INFO("DMP traj size:    %d", traj.points.size());
                }
                else {
                    ROS_INFO("clientPlan services call failed ! ");
                }
            }
            ROS_INFO("DMP services call successful ! ");

            //        cout<<traj.points.size()<<endl;
        // save_traj(traj, origin.pose.position.z);
            geometry_msgs::PoseStamped cur;
            geometry_msgs::PoseStamped des;

            Eigen::AngleAxisd V1(M_PI / 2, Eigen::Vector3d(0, 1, 0));
            Eigen::Matrix3d rotate = V1.matrix();
            Eigen::Quaterniond quat(rotate);

            //    while(ros::ok()){
            for (int i = 0; i < traj.points.size(); ++i) {

                // cout<<"---------------"<<traj.times[i]<<"---------------"<<endl;
                path.header.stamp = ros::Time::now();
                this_pose_stamped.header.stamp = ros::Time::now();
                this_pose_stamped.pose.position.x = traj.points[i].positions[0];
                this_pose_stamped.pose.position.y = traj.points[i].positions[1];
                // this_pose_stamped.pose.position.z = origin.pose.position.z;
                this_pose_stamped.pose.position.z = 0.0530;
                // this_pose_stamped.pose.orientation = origin.pose.orientation;
                this_pose_stamped.pose.orientation.x = quat.x();
                this_pose_stamped.pose.orientation.y = quat.y();
                this_pose_stamped.pose.orientation.z = quat.z();
                this_pose_stamped.pose.orientation.w = quat.w();

                path.poses.push_back(this_pose_stamped);
                // iiwa_pub.publish(this_pose_stamped);
                // pathDes.header.stamp = ros::Time::now();
                // this_pose_stamped.pose.position.x = cos(0.2*M_PI*time);
                // this_pose_stamped.pose.position.y = cos(0.5*M_PI*time);
                // this_pose_stamped.pose.position.z = 0;
                // pathDes.poses.push_back(this_pose_stamped);
                // pathDesPub.publish(pathDes);
                
            }
            my_index = my_index_temp;
            pathPub.publish(path);
        }
        
        rate.sleep();
    }
        ROS_INFO("DMP path end!!!");
            //    }
}
