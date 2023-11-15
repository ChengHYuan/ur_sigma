/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:12:59
 */

#ifndef _EposMotor_HPP
#define _EposMotor_HPP

#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "epos_driver/Device.hpp"
#include "epos_driver/control/ControlModeBase.hpp"
#include "epos_msgs/MotorState.h"


class EposMotor {
public:
    EposMotor();
    virtual ~EposMotor();

    void init(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);

    epos_msgs::MotorState read();
    void write(const double position, const double velocity, const double current);

private:
    void initEposDeviceHandle(ros::NodeHandle &motor_nh);//创建epos句柄
    void initDeviceError();
    void initProtocolStackChanges(ros::NodeHandle &motor_nh);//设置通信波特率，超时时间
    void initControlMode(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh);//设置运行模式
    void initEncoderParams(ros::NodeHandle &motor_nh);
    void initPVC(ros::NodeHandle &motor_nh);
    void initProfilePosition(ros::NodeHandle &motor_nh);//设置PPM模式下的参数
    void initProfileVelocity(ros::NodeHandle &motor_nh);//设置PVM模式下的参数
    void initMiscParams(ros::NodeHandle &motor_nh);

    double ReadPosition();
    double ReadVelocity();
    double ReadCurrent();

    void writeCallback(const epos_msgs::MotorState::ConstPtr &msg);

private:

    typedef std::shared_ptr<ControlModeBase> ControlModePtr;
    typedef std::map<std::string, ControlModePtr> ControlModeMap;

    std::string m_motor_name;

    NodeHandle m_epos_handle;
    ControlModePtr m_control_mode;
    ControlModeMap m_control_mode_map;

    double m_position;
    double m_velocity;
    double m_effort;
    double m_current;

    ros::Publisher m_state_publisher;
    ros::Subscriber m_state_subscriber;

    int m_max_qc;
    int ratio;
    bool m_use_ros_unit;
};

#endif // _EposMotor_HPP
