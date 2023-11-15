/**
 * @file   maxon_bringup
 * @brief  
 * @author arwtyxouymz
 * @date   2019-05-24 17:49:41
 */

#include <ros/ros.h>
#include <vector>
#include <string>
#include "epos_driver/EposManager.hpp"
#include "epos_msgs/MotorState.h"

//角度控制测试
double joint_test(int &step,int &step_num)
{
    double joint_set;
    joint_set=M_PI/6*sin(2*step*M_PI/step_num);
    return joint_set;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "one_motor_raiseup");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::vector<std::string> motor_names;
    if (!private_nh.getParam("motor_names", motor_names)) {
        ROS_FATAL("Failed to load motor_names");
        return 1;
    }

    ros::Rate sleep_rate(50);
    EposManager manager;
    if (!manager.init(nh, private_nh, motor_names))
    {
        ROS_FATAL("Failed to initialize EposManager");
        return 1;
    }
    ROS_INFO("Motors Initialized");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int step_num=100;
    int step=0;
    boost::shared_ptr<epos_msgs::MotorStates> msg(new epos_msgs::MotorStates);
    while (ros::ok()) {

        msg->states[0].position=joint_test(step,step_num);
        
        manager.write(msg);
        manager.read();
        
        sleep_rate.sleep();
    }

    spinner.stop();

    return 0;
}