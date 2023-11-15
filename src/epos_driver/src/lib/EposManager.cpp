/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 */

#include "epos_driver/EposManager.hpp"
#include "epos_msgs/MotorState.h"
#include "epos_msgs/MotorStates.h"
#include <boost/foreach.hpp>

#include<iostream>
#include<string>
#include <fstream>
#include <thread>
using namespace std;
/**
 * @brief Constructor
 */
EposManager::EposManager() = default;

/**
 * @brief Destructor
 */
EposManager::~EposManager() = default;


/**
 * @brief Initialize function
 *
 * @param root_nh 
 * @param motors_nh
 * @param motor_names
 *
 * @return 
 */
bool EposManager::init(ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
        const std::vector<std::string> &motor_names)
{
    BOOST_FOREACH (const std::string &motor_name, motor_names)
    {
        ROS_INFO_STREAM("Loading Epos: " << motor_name);
        // Copy constructor => ns = motors_nh's namespace + / + motor_name
        ros::NodeHandle motor_nh(motors_nh, motor_name);///更改命名空间使得电机加载自己相对应的配置

        std::shared_ptr<EposMotor> motor(new EposMotor());
        motor->init(root_nh, motor_nh, motor_name);
        m_motors.push_back(motor);
    }

    m_all_motor_publisher = motors_nh.advertise<epos_msgs::MotorStates>("get_all_states", 100);//创建发布器  100表示缓冲区大小
    m_all_motor_subscriber = motors_nh.subscribe("set_all_states", 100, &EposManager::write, this);

    data.resize(m_motors.size());
    return true;
}

void EposManager::read()
{
    fstream out("/home/zh/catkin_ws/src/epos_control/data/data.txt",ios::out|ios::app);

    epos_msgs::MotorStates msg;
    for (int i = 0; i < m_motors.size(); i++) {
        
        epos_msgs::MotorState state = m_motors[i]->read();
        data[i].push_back(state);
        out<<fixed<<setprecision(4)<<i+1<<"\t\t"<<state.position<<"\t\t"<<state.velocity<<"\t\t"<<state.current<<std::endl; 
        
        msg.states.push_back(state);
    }
    out.close();
    return;
}

void EposManager::read(int i)
{
    std::string str{"/home/zh/catkin_zh/src/epos_control/data/data"};
    str+=to_string(i+1);
    str+=".txt";
    fstream out(str.c_str(),ios::out|ios::app);

    clock_t  time=ros::Time::now().toNSec();
    double t = (double)time/ 1e6;

    epos_msgs::MotorState state = m_motors[i]->read();
    data[i].push_back(state);
    out<<fixed<<setprecision(12)<<t<<"\t\t"<<state.position<<"\t\t"<<state.velocity<<"\t\t"<<state.current<<std::endl;
    out.close();
    return;
}

vector<vector<epos_msgs::MotorState>> EposManager::readData(){
    return data;
}

void EposManager::write(const epos_msgs::MotorStates::ConstPtr& msg)
{
    // for (int i = 0; i < m_motors.size(); i++) {
    //     epos_msgs::MotorState state = msg->states[i];
    //     ROS_DEBUG_STREAM("Send: " << state.position);
    //     m_motors[i]->write(state.position, state.velocity, state.current);
    // }
    vector<thread> mythreads;
    for (int i = 0; i < m_motors.size(); i++) {
        epos_msgs::MotorState state = msg->states[i];
        ROS_DEBUG_STREAM("Send: " << state.position);
        // ROS_INFO("Flag!!!!!!!!");
        mythreads.push_back(thread(&EposMotor::write, m_motors[i], state.position, state.velocity, state.current));
    }
    
    for (int i = 0; i < m_motors.size(); i++) {
        mythreads[i].join();
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void EposManager::write(const epos_msgs::MotorState::ConstPtr& state,int index)
{
    ROS_DEBUG_STREAM("Send: " << state->velocity);
    //ROS_DEBUG_STREAM("Send: " << state->position);
    m_motors[index]->write(state->position, state->velocity, state->current);

}

int EposManager::size(){
    return m_motors.size();
}

std::vector<double> EposManager::get_joints() {
//   read();
  epos_msgs::MotorState msg;
  std::vector<double> joints_of_all;
    for (int i = 0;i < m_motors.size();++i) {
    msg = m_motors[i]->read();
    joints_of_all.push_back(msg.position);
  }
//   vector<thread> read_threads;
//   for (int i = 0;i < m_motors.size();++i) {
//     read_threads.push_back(thread(&EposMotor::read, m_motors[i]));
//   }
//   joints_of_all.push_back(msg.position);

  return joints_of_all;
}
