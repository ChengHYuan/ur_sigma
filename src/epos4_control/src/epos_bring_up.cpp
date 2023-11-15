#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include "maxon_epos_driver/EposManager.hpp"
#include "maxon_epos_msgs/MotorState.h"
#include "maxon_epos_msgs/MotorStates.h"
#include <Eigen/Dense>
#define _USE_MATH_DEFINES
#include <math.h>


Eigen::Matrix<double,3,3> eye;
std::vector<double> decouple_param;
Eigen::Matrix3d t_tool;

Eigen::Matrix3d epos_kine(maxon_epos_msgs::MotorStates::Ptr &msg){
    std::vector<double> motor_position;
    std::vector<double> joints(3);
    //获取当前各个电机转动角度
    for(int i=0;i<msg->states.size();++i){
        motor_position[i]=msg->states[i].position;
    }
    //计算每个关节角度
    joints[0]=motor_position[0];
    joints[1]=motor_position[1];
    joints[2]=(motor_position[2]+motor_position[3])/2;

    Eigen::Matrix<double,3,3> R_tool;
    std::vector<Eigen::Matrix<double,3,3>> R_s;

    //计算旋转矩阵
    eye<<1,0,0,
         0,1,0,
         0,0,1;

    R_s[0]<<1,0,0,
            0,cos(joints[0]),-sin(joints[0]),
            0,sin(joints[0]),cos(joints[0]);

    R_s[1]<<cos(joints[1]),0,-sin(joints[1]),
            0,1,0,
            -sin(joints[1]),0,cos(joints[1]);

    R_s[2]<<cos(joints[2]),-sin(joints[2]),0,
            sin(joints[2]),cos(joints[2]),0,
            0,0,1;

    //给出正运动学结果
    R_tool=eye*R_s[0]*R_s[1]*R_s[2];

    return R_tool;
}

std::vector<double> ikine(Eigen::Matrix3d &r_tool){

}

//角度控制测试
std::vector<double> joint_test(int &step,int &step_num)
{
    std::vector<double> joints_set;
    joints_set.push_back(M_PI/6*sin(2*step*M_PI/step_num));
    joints_set.push_back(M_PI/6*sin(2*step*M_PI/step_num));
    joints_set.push_back(M_PI/6*sin(2*step*M_PI/step_num));
    joints_set.push_back(M_PI/6*sin(-2*step*M_PI/step_num));
    return joints_set;

}

//设置关节角位置（逆运动学）
void setMototPosition(maxon_epos_msgs::MotorStates::Ptr &msg){
    std::vector<double> joints_target=ikine(t_tool);
    for(int i=0;i<msg->states.size();++i){
            msg->states[i].position=joints_target[i];
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "maxon_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::vector<std::string> motor_names;

    if (!private_nh.getParam("motor_names", motor_names)) {
        ROS_FATAL("Failed to load motor_names");
        return 1;
    }

    EposManager manager;//创建管理对象

    if (!manager.init(nh, private_nh, motor_names))
    {
        ROS_FATAL("Failed to initialize EposManager");
        return 1;
    }
    ROS_INFO("Motors Initialized");

    ros::Rate sleep_rate(50);

    boost::shared_ptr<maxon_epos_msgs::MotorStates> msg_sent(new maxon_epos_msgs::MotorStates());

    int step=0;
    int step_num=100;
    std::vector<double> joints(4);
    while(ros::ok()){

        //为每个关节角赋值
        joints=joint_test(step,step_num);
        for(int i=0;i<msg_sent->states.size();++i){
            msg_sent->states[i].position=joints[i];
        }
        //发送关节角信息
        manager.write(msg_sent);

        manager.read();

        sleep_rate.sleep();

        step++;
    }

    ROS_INFO("The End !");
    ros::spin();

    return 0;

}