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

//角度控制测试
double joint_test(int &step,int &step_num)
{
    double joint_set;
    //joint_set=M_PI/2*sin(2*step*M_PI/step_num);
    //joint_set=2*M_PI;
    joint_set=0;
    return joint_set;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "maxon_bringup");
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
    boost::shared_ptr<epos_msgs::MotorStates> msg(new epos_msgs::MotorStates());
    epos_msgs::MotorState state;
    state.position=0;
    msg->states.push_back(state);

    while (ros::ok()) {

        state.position=joint_test(step,step_num);
        ROS_INFO("motor set position is %f",state.position);

        msg->states[0]=state;
        
        manager.write(msg);
        manager.read();
        
        sleep_rate.sleep();
        step++;
    }
    // while (ros::ok()) {

    //     // auto start=std::chrono::steady_clock::now();

    //     manager.read();
    //     // auto finish=std::chrono::steady_clock::now();

    //     // auto duration=std::chrono::duration_cast<std::chrono::milliseconds>(finish-start);
        
    //     // ROS_INFO("duration between start and finish is %f ms",duration.count());
    //     sleep_rate.sleep();
    // }

    spinner.stop();

    return 0;
}
