/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:08:23
 */

#ifndef _EposManager_HPP
#define _EposManager_HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include "epos_driver/EposMotor.hpp"
#include "epos_msgs/MotorStates.h"

using namespace std;

class EposManager {

public:
    EposManager();
    virtual ~EposManager();

    bool init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
            const std::vector<std::string> &motor_names);

    void read();
    void read(int i);

    std::vector<std::vector<epos_msgs::MotorState>> readData();

    void write(const epos_msgs::MotorStates::ConstPtr& msg);
    void write(const epos_msgs::MotorState::ConstPtr& state,int index);

    int size();

    std::vector<double> get_joints();

private:
    std::vector<std::shared_ptr<EposMotor>> m_motors;
    ros::Publisher m_all_motor_publisher;
    ros::Subscriber m_all_motor_subscriber;

    std::vector<vector<epos_msgs::MotorState>> data;
};

#endif // _EposManager_HPP
