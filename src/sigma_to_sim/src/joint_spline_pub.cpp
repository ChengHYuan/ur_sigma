#include <iostream>
#include <string>
#include "ros/ros.h"
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include "joint_spline.hpp"
#include <memory>

using namespace std;

//创建样条对象
shared_ptr<IIwaSpline> spline_ptr;

void call_back(const iiwa_msgs::JointPosition& msg) {
    bool begin = spline_ptr->is_position_full;
    spline_ptr->iiwa_joint_spline(msg);
    if (!begin && spline_ptr->is_position_full) {
        ROS_INFO("position stack is full!!!");
    }
}

std_msgs::Float64MultiArray tran(iiwa_msgs::JointPosition& msgs) {
    std_msgs::Float64MultiArray answer;
    answer.data.resize(7);
    answer.data[0] = msgs.position.a1;
    answer.data[1] = msgs.position.a2;
    answer.data[2] = msgs.position.a3;
    answer.data[3] = msgs.position.a4;
    answer.data[4] = msgs.position.a5;
    answer.data[5] = msgs.position.a6;
    answer.data[6] = msgs.position.a7;
    return answer;
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "joint_spline_pub");

    ros::NodeHandle nh;

    spline_ptr =make_shared<IIwaSpline>();


    ros::Subscriber sub_of_jp = nh.subscribe("/iiwa/command/JointPosition_2", 2,call_back);//获取目标关节角
    // ros::Publisher pub_of_jpv = nh.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity", 1);//发布计算后的关节角和速度
    // ros::Publisher pub_of_jp = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointDirectPosition", 1);//发布计算后的关节角
    // ros::Publisher pub_of_jp = nh.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 2);//发布计算后的关节角
    ros::Publisher pub_of_jp = nh.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 1);//发布计算后的关节角
    ros::Rate rate(400);
    ROS_INFO("spline node start!!!");
    while (!spline_ptr->is_position_full) {
        ros::spinOnce();
    }
    int step = 0;
    while (ros::ok())
    {
        if (spline_ptr->is_position_full&&!spline_ptr->iiwa_position_velocity.empty()) {
            // cout << "point_size: " << iiwa_spline.joint_position_velocity.size() << endl;
            // iiwa_msgs::JointPositionVelocity jpv = spline_ptr->get_iiwa_position_velocity();
            // jpv.header.stamp = ros::Time::now();
            // pub_of_jpv.publish(jpv);
            
            // iiwa_msgs::JointPosition jp = spline_ptr->get_iiwa_position();
            std_msgs::Float64MultiArray jp = spline_ptr->get_iiwa_fri_position();
            // jp.header.stamp = ros::Time::now();
            // std_msgs::Float64MultiArray answer = tran(jp);
            
            pub_of_jp.publish(jp);
        }
        step++;
        if (step == spline_ptr->EXPEND_EACH) {
            ros::spinOnce();
            step = 0;
        }
        rate.sleep();
    }
    if (ros::isShuttingDown) {
        ROS_INFO("end!!!!");
    }
    return 0;

}