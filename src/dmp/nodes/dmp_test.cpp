//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include<std_msgs/String.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "tinyxml2.h"
#include <iiwa_msgs/CartesianPose.h>

using namespace std;
using namespace tinyxml2;

geometry_msgs::PoseStamped origin;


void save_traj(dmp::DMPTraj &traj,double &z) {
    ofstream ofs; //定义流对象
    ofs.open("/home/chy/vrep_path/text01.txt", ios::out);//以写的方式打开文件
    for (int i = 0;i < traj.points.size();++i) {
        ofs << traj.points[i].positions[0] << " " << traj.points[i].positions[1] << " " << z << endl;
    }
    ROS_INFO("path is saved!!!!");
    ofs.close();
}


void cos_generate(std::vector<dmp::DMPPoint>& points, double time_step) {
    for(int i=0;i<100;i++){
        double t = i*time_step;
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(cos(0.2*M_PI*t));
        xpoint.velocities.push_back(-0.2*M_PI*sin(0.2*M_PI*t));
        // y axis
        xpoint.positions.push_back(cos(0.5*M_PI*t));
        xpoint.velocities.push_back(-0.1*M_PI*sin(0.5*M_PI*t));
        points.push_back(xpoint);
        // std::cout<< cos(0.2*M_PI*t) <<" , "<< -0.2*M_PI*sin(0.2*M_PI*t) <<std::endl;
    }
}

void peg_transfer_generate(std::vector<dmp::DMPPoint> &points,double time_step){
    for(int i=0;i<25;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(0);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(i*0.04);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<50;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(i*0.1);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(1);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<25;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(5);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(1-0.04*i);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
}

void trangle_demo_generate(std::vector<dmp::DMPPoint> &points,double time_step){
    for(int i=0;i<100;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(0);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(i*0.05);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<50;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(i*0.1);
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(5-i*0.05);
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
    for(int i=0;i<25;i++){
        dmp::DMPPoint xpoint;
        xpoint.positions.push_back(5+3*sin(i*M_PI/50));
        xpoint.velocities.push_back(0.01);
        xpoint.positions.push_back(2.5+2*sin(i*M_PI/150));
        xpoint.velocities.push_back(0.01);
        points.push_back(xpoint);
    }
}

void time_generate(std::vector<dmp::DMPPoint> points, std::vector<double>& times, double time_step) {
    for(int i=0;i<points.size();i++){
        times.push_back(i*time_step);
    }
}


void get_dmp_demo_from_bag(std::vector<dmp::DMPPoint> &points,double &time_step){
    rosbag::Bag demo_bag;
    // demo_bag.open("/home/chy/bagfile/path01.bag", rosbag::bagmode::Read);//以写的模式 打开bag文件的绝对地址
    demo_bag.open("/home/chy/bagfile/end_path_3.bag", rosbag::bagmode::Read);
    ROS_INFO("------------file is loaded------------");

    vector<string> demo_topic;
    demo_topic.push_back(string("/iiwa/state/CartesianPose_end"));
    rosbag::View view(demo_bag, rosbag::TopicQuery(demo_topic));//创建一个迭代器，指向bag中的元素
    rosbag::View::iterator it = view.begin();

    vector<double> local_velocity = { 0,0,0 };
    double time_duration;
    geometry_msgs::PoseStamped pose_before;
    geometry_msgs::PoseStamped pose_local;

    for (;it != view.end();++it) {
        auto mes=*it;//获取一帧的内容
        string topic_local=mes.getTopic();
        if (topic_local == "/iiwa/state/CartesianPose_end") {
            iiwa_msgs::CartesianPose::ConstPtr pose_msg = mes.instantiate<iiwa_msgs::CartesianPose>();
            if (pose_msg != NULL) {
                pose_local = pose_msg->poseStamped;
                if (it == view.begin()) {
                    origin = pose_msg->poseStamped;
                }
                dmp::DMPPoint demo;
                demo.positions.push_back(pose_local.pose.position.x);//将x y z 位置发送给
                demo.positions.push_back(pose_local.pose.position.y);
                if (demo.positions.size() == 0) {
                    pose_before = pose_local;
                    local_velocity[0] = 0;
                    local_velocity[1] = 0;
                    local_velocity[2] = 0;
                }
                else {
                    time_duration = pose_local.header.stamp.toSec() - pose_before.header.stamp.toSec();//通过差分计算速度
                    // if (time_step == 0) {
                    //     time_step = time_duration;
                    //     ROS_INFO("time step is %f",time_step);
                    // }
                    local_velocity[0] = (pose_local.pose.position.x - pose_before.pose.position.x) / time_duration;
                    local_velocity[1] = (pose_local.pose.position.y - pose_before.pose.position.y) / time_duration;
                    local_velocity[2] = (pose_local.pose.position.z - pose_before.pose.position.z) / time_duration;
                }
                
                demo.velocities.push_back(local_velocity[0]);
                demo.velocities.push_back(local_velocity[1]);

                points.push_back(demo);
                pose_before = pose_local;

                // demo.positions.push_back(pose_msg->poseStamped.pose.position.z);
                // demo.velocities.push_back();
            }
            else{
                ROS_INFO("topic message is empty!!!");
            }
        }

        // if (points.size() / 2 >= 500) {
        //     break;
        // }
    }
    demo_bag.close();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle n;

    XMLDocument doc;
    doc.LinkEndChild(doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\""));
    doc.LinkEndChild(doc.NewComment("this is a DMP library data"));
    XMLElement* root=doc.NewElement("DMPs");
    doc.InsertEndChild(root);

    ros::ServiceClient clientLfd = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
    ros::ServiceClient clientPlan= n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
    ros::ServiceClient clientActive = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");

    
    // ros::Publisher trajPub = n.advertise<dmp::DMPPointStamp>("dmp/trajectory",100);
    // ros::Publisher trajDesPub = n.advertise<dmp::DMPPointStamp>("dmp/des_trajectory",100);
    ros::Publisher trajPub = n.advertise<geometry_msgs::PoseStamped>("dmp/trajectory",100);
    ros::Publisher trajDesPub = n.advertise<geometry_msgs::PoseStamped>("dmp/des_trajectory",100);
    ros::Publisher pathPub = n.advertise<nav_msgs::Path>("xy/trajectory",10, true);
    ros::Publisher pathDesPub = n.advertise<nav_msgs::Path>("xy/des_trajectory", 10, true);
    ros::Publisher iiwa_pub = n.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose_origin", 1,true);

    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    nav_msgs::Path pathDes;
    pathDes.header.frame_id="world";

    dmp::LearnDMPFromDemo srv;
    std::vector<dmp::DMPPoint> points;
    std::vector<double> times;

    double time_step=0.008;

    peg_transfer_generate(points, time_step);
    // trangle_demo_generate(points, time_step);
    // get_dmp_demo_from_bag(points,time_step);
    // cos_generate(points,time_step);//生成轨迹点（cos）
    time_generate(points,times,time_step);//生成时间点

    ROS_INFO("demo points size:  %d",points.size());


    double k_gain = 25;
    // double k_gain = 25;
    // double d_gain = k_gain/4;
    double d_gain = k_gain/4;

    srv.request.k_gains = std::vector<double>{k_gain,k_gain};
    srv.request.d_gains = std::vector<double>{d_gain,d_gain};

    srv.request.num_bases = 100;
    srv.request.demo.points = points;
    srv.request.demo.times = times;

    string dim = to_string(srv.request.k_gains.size());
    root->SetAttribute("dim",dim.c_str());


    if(clientLfd.call(srv)){
        // sleep(5);
        double tau = srv.response.tau;
        XMLElement* tau_element=doc.NewElement("tau");
        tau_element->LinkEndChild( doc.NewText( to_string(tau).c_str() ));
        root->LinkEndChild(tau_element);
        
        std::vector<dmp::DMPData>  dmpList = srv.response.dmp_list;
        for(int i=0;i<dmpList.size();++i){
            auto dmpData = dmpList[i];
            XMLElement* dmp_dataElement = doc.NewElement( "dmp" );
            string id = to_string(i);
            dmp_dataElement->SetAttribute("id",id.c_str());
            XMLElement* kElement = doc.NewElement( "kgain" );
            kElement->LinkEndChild( doc.NewText( to_string(dmpData.k_gain).c_str() ));
            XMLElement* dElement = doc.NewElement( "dgain" );
            dElement->LinkEndChild( doc.NewText( to_string(dmpData.d_gain).c_str() ));
            dmp_dataElement->SetAttribute("id",id.c_str());
            dmp_dataElement->LinkEndChild(kElement);
            dmp_dataElement->LinkEndChild(dElement);
            XMLElement* weightsElement = doc.NewElement( "weights" );
            weightsElement->SetAttribute("nums", to_string(dmpData.weights.size()).c_str());
            for (int j = 0;j<dmpData.weights.size();++j) {
                XMLElement* weightElement = doc.NewElement( "w" );
                weightElement->SetAttribute("id", to_string(j).c_str());
                weightElement->LinkEndChild( doc.NewText( to_string(dmpData.weights[j]).c_str() ));
                weightsElement->InsertEndChild( weightElement );
            }
            dmp_dataElement->LinkEndChild(weightsElement);
            root->LinkEndChild(dmp_dataElement);
        }
        XMLPrinter printer;
        doc.Print(&printer);
        cout<< printer.CStr() << endl;
        doc.SaveFile("myXML.xml");

//        std::vector<double> w = dmpList[0].weights;
//        for(double i:w){
//            std::cout<<i<<std::endl;
//        }


        double x_0 = 0,y_0 = 0;
        double x_goal = 0, y_goal = 0;

        dmp::SetActiveDMP srvSetActive;
        dmp::DMPTraj traj;
        srvSetActive.request.dmp_list = dmpList;
        if(clientActive.call(srvSetActive)){
            dmp::GetDMPPlan srvPlan;
            srvPlan.request.x_0.push_back(points[0].positions[0]);srvPlan.request.x_0.push_back(points[0].positions[1]);//设置起始位置
            // srvPlan.request.x_0.push_back(0.6645);srvPlan.request.x_0.push_back(0.0375);//设置起始位置
            srvPlan.request.x_dot_0.push_back(0);srvPlan.request.x_dot_0.push_back(0);
            srvPlan.request.t_0 = 0;
            srvPlan.request.goal.push_back(points[points.size() - 1].positions[0]);srvPlan.request.goal.push_back(points[points.size() - 1].positions[1]);//设置目标位置
            // srvPlan.request.goal.push_back(0.6645);srvPlan.request.goal.push_back(-0.0357);
            srvPlan.request.goal_thresh.push_back(0.01);srvPlan.request.goal_thresh.push_back(0.01);
            // srvPlan.request.seg_length = 20;
            srvPlan.request.seg_length = -1;
            srvPlan.request.tau = tau;
            srvPlan.request.dt = 0.01;
            srvPlan.request.integrate_iter = 1;
            if(clientPlan.call(srvPlan) && srvPlan.response.at_goal){
                traj = srvPlan.response.plan;
                ROS_INFO("DMP traj size:    %d",traj.points.size());
            }else{
                ROS_INFO("clientPlan services call failed ! ");
            }
        }
        ROS_INFO("DMP services call successful ! ");
        //        cout<<traj.points.size()<<endl;

        save_traj(traj, origin.pose.position.z);
        
        geometry_msgs::PoseStamped cur;
        geometry_msgs::PoseStamped des;
        

        ros::Rate rate(250);
    //    while(ros::ok()){
            for (int i = 0; i < traj.points.size(); ++i) {
                
                // cout<<"---------------"<<traj.times[i]<<"---------------"<<endl;
                cur.header.stamp= ros::Time::now();
                cur.pose.position.x = traj.points[i].positions[0];
                cur.pose.position.y = traj.points[i].positions[1];
                trajPub.publish(cur);

                if (i < points.size()) {
                    double time = i * 0.01;
                    des.header.stamp = ros::Time::now();
                    des.pose.position.x = points[i].positions[0];
                    des.pose.position.y = points[i].positions[1];
                    trajDesPub.publish(des);
                }
                
                path.header.stamp = ros::Time::now();
                this_pose_stamped.header.stamp = ros::Time::now();
                this_pose_stamped.pose.position.x = traj.points[i].positions[0];
                this_pose_stamped.pose.position.y = traj.points[i].positions[1];
                this_pose_stamped.pose.position.z = origin.pose.position.z;
                this_pose_stamped.pose.orientation = origin.pose.orientation;
                path.poses.push_back(this_pose_stamped);
                pathPub.publish(path);
                iiwa_pub.publish(this_pose_stamped);
                // pathDes.header.stamp = ros::Time::now();
                // this_pose_stamped.pose.position.x = cos(0.2*M_PI*time);
                // this_pose_stamped.pose.position.y = cos(0.5*M_PI*time);
                // this_pose_stamped.pose.position.z = 0;
                // pathDes.poses.push_back(this_pose_stamped);
                // pathDesPub.publish(pathDes);
                rate.sleep();
            }
            ROS_INFO("DMP path end!!!");
            //    }
    }else {
        ROS_INFO("Failled call DMP services! ");
    }
    return 0;
}