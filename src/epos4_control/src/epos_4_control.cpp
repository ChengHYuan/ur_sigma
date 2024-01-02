#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include "epos_driver/EposManager.hpp"
#include "epos_msgs/MotorState.h"
#include "epos_msgs/MotorStates.h"
#include <Eigen/Dense>
#include<sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#define _USE_MATH_DEFINES
#include <math.h>



class Epos4Control{
private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_private_nh;
    ros::Subscriber m_sub;
    ros::Subscriber m_sub_vel;
    
    ros::Publisher m_pub;
    boost::shared_ptr<epos_msgs::MotorStates> m_msg_sent;
    EposManager m_manager;
    std::vector<std::string> motor_names;


public:
    Epos4Control(ros::NodeHandle _nh,ros::NodeHandle _private_nh){
        m_private_nh=_private_nh;
        m_nh = _nh;
        if (!m_private_nh.getParam("motor_names", motor_names)) {
            ROS_FATAL("Failed to load motor_names");
        }
        m_manager;
        if (!m_manager.init(m_nh, m_private_nh, motor_names))
        {
            ROS_FATAL("Failed to initialize EposManager");
        }
        m_sub = m_private_nh.subscribe("/endoWrist/command/JointPosition", 1, &Epos4Control::callback, this);
        //m_sub_vel = m_private_nh.subscribe("/endoWrist/command/JointVelocity", 1, &Epos4Control::callback_vel, this);
        m_pub = m_private_nh.advertise<std_msgs::Float32MultiArray>("/endoWrist/state/CartesianPose", 1);
        m_msg_sent = boost::shared_ptr<epos_msgs::MotorStates>(new epos_msgs::MotorStates());
        //将消息按照电机数量初始化
        init_msg(motor_names.size());
        m_manager.read();

        ROS_INFO("Motors Initialized");
    }
    void callback(const std_msgs::Float32MultiArray& msg);
    void callback_vel(const std_msgs::Float32MultiArray& msg);
    
    void read_all();

    void get_num_of_motor();

    //退出时将所有的电机复原
    void return_to_zero();
    
    //测试每个电机的序号
    void test_for_motor(int step, int step_num);
    void test_for_motor_2(int step, int step_num);

    void init_msg(int names_size);

    //将四个关节进行解耦
    void decouple_joints(const std::vector<float>& joints_msgs);

    //关节速度解耦
    void decouple_joints_vel(const std::vector<float>& joints_msgs);

};


void Epos4Control::get_num_of_motor() {
    // ROS_INFO("num of moters is : %d",m_manager.size());
}

void Epos4Control::return_to_zero(){
    for(int i=0;i<m_msg_sent->states.size();++i){
        m_msg_sent->states[i].position = 0;
    }
    m_manager.write(m_msg_sent);
}

//将四个关节进行解耦
void Epos4Control::decouple_joints(const std::vector<float>& joints_msgs) {
    Eigen::MatrixXd trans(4, 4), joints_desire(4, 1), joints_motor(4, 1);
    
    double k_p1 = -1, k_p2 = -0.6, k_p3 = 0.6, k_y1 = 1, k_y2 = -1, k_r = 1;
    
    double k_offset1 = 0, k_offset2 = 0, k_offset3 = 0, k_offset4 = 0;
    
    trans << k_r, 0, 0, k_offset3,
        0, k_p1, 0, k_offset4,
        0, k_p2, k_y1, k_offset1,
        0, k_p3, k_y2, k_offset2;
    joints_desire << joints_msgs[0], joints_msgs[1], joints_msgs[2], 1;
    joints_motor = trans * joints_desire;

    // cout << "joints_angles: " << joints_motor << endl;
    
    m_msg_sent->states[1].position = joints_motor(3) + joints_msgs[3] / 2;
    m_msg_sent->states[2].position = joints_motor(2) + joints_msgs[3] / 2;
    m_msg_sent->states[3].position = joints_motor(1);
    m_msg_sent->states[0].position = joints_motor(0);

}

//将四个关节进行解耦(速度方面)
void Epos4Control::decouple_joints_vel(const std::vector<float>& joints_msgs) {
    Eigen::MatrixXd trans(4, 4), joints_desire(4, 1), joints_motor(4, 1);

    double k_p1 = -1, k_p2 = 0, k_p3 = 0, k_y1 = 1, k_y2 = -1, k_r = 1;
    
    double k_offset1 = 0, k_offset2 = 0, k_offset3 = 0, k_offset4 = 0;
    
    trans << k_r, 0, 0, k_offset3,
        0, k_p1, 0, k_offset4,
        0, k_p2, k_y1, k_offset1,
        0, k_p3, k_y2, k_offset2;
    joints_desire << joints_msgs[0], joints_msgs[1], joints_msgs[2], 1;
    joints_motor = trans * joints_desire;

    // cout << "joints_angles: " << joints_motor << endl;

    m_msg_sent->states[0].velocity = joints_motor(3);
    m_msg_sent->states[3].velocity = joints_motor(2);
    m_msg_sent->states[1].velocity = joints_motor(1);
    m_msg_sent->states[2].velocity = joints_motor(0);
}

void Epos4Control::callback(const std_msgs::Float32MultiArray& msg) {
    //ROS_INFO("the time is %f",m_time.now());
    if (m_msg_sent->states.size() == 1) {
        // ROS_INFO("FLAG!!!!!!!!!!!!!!!!");
        m_msg_sent->states[0].position = msg.data[3];
    }
    else {
        decouple_joints(msg.data);
        // m_msg_sent->states[0].position = msg.data[0];
        // m_msg_sent->states[1].position = msg.data[1];
        // m_msg_sent->states[2].position = msg.data[2];
        // m_msg_sent->states[3].position = msg.data[2];
    }
    
    // decouple_joints(msg.data);
    double before = ros::Time::now().toSec();

    // std::thread th1(&EposManager::write, &m_manager, std::ref(m_msg_sent));
    m_manager.write(m_msg_sent);
    // read_all();
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));

    double now = ros::Time::now().toSec();
    cout << "duration: " << (now - before) * 1000 <<" ms"<< endl;

}


void Epos4Control::callback_vel(const std_msgs::Float32MultiArray& msg) {
    //ROS_INFO("the time is %f",m_time.now());
    if (m_msg_sent->states.size() == 1) {
        m_msg_sent->states[0].velocity = msg.data[0];
    }
    else {
        decouple_joints(msg.data);
        // m_msg_sent->states[0].position = msg.data[0];
        // m_msg_sent->states[1].position = msg.data[1];
        // m_msg_sent->states[2].position = msg.data[2];
        // m_msg_sent->states[3].position = msg.data[2];
    }
  
    double before = ros::Time::now().toSec();

    // std::thread th1(&EposManager::write, &m_manager, std::ref(m_msg_sent));
    m_manager.write(m_msg_sent);
    // read_all();
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));s

    double now = ros::Time::now().toSec();
    cout << "duration: " << (now - before) * 1000 <<" ms"<< endl;
}

void Epos4Control::test_for_motor(int step, int step_num) {//测试每一个电机是哪一个
    int num = 3;

    m_msg_sent->states[num].position = M_PI / 24 * sin(10 * step * M_PI / step_num);
    // m_msg_sent->states[num].position = M_PI/12;

    // cout << "sent position: " << M_PI / 6 * sin(2 * step * M_PI / step_num) << endl;
    m_manager.write(m_msg_sent);
}

void Epos4Control::test_for_motor_2(int step, int step_num) {//测试每一个电机是哪一个
    // int num = 0;
    // cout << "size: " << m_msg_sent->states.size() << endl;
    m_msg_sent->states[0].position = M_PI / 3 * sin(20*step * M_PI / 360);
    m_msg_sent->states[1].position = M_PI / 3 * sin(20 * step * M_PI / 360);
    m_msg_sent->states[2].position = M_PI / 3 * sin(20 * step * M_PI / 360);
    m_msg_sent->states[3].position = M_PI / 3 * sin(20*step * M_PI / 360);
    // cout << "sent position: " << M_PI / 6 * sin(2 * step * M_PI / step_num) << endl;
    m_manager.write(m_msg_sent);
    
}

void Epos4Control::init_msg(int names_size){
    epos_msgs::MotorState state;
    state.position=0;
    for(int i=0;i<names_size;++i){
        m_msg_sent->states.push_back(state);
    }
}

//读取所有电机角度
void Epos4Control::read_all() {
    std_msgs::Float32MultiArray joints;
    joints.data.resize(4);
    std::vector<double> joints_vector;
    joints_vector = m_manager.get_joints();
    // for (int i = 0;i < joints_vector.size();++i) {
    //     joints.data.push_back(joints_vector[i]);
    // }
    
    joints.data[0] = joints_vector[0];
    joints.data[1] = joints_vector[1];
    joints.data[2] = -joints_vector[2];
    joints.data[3] = joints_vector[3];


    m_pub.publish(joints);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "maxon_4_control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // ROS_INFO("------------------FLAG--------------");

    Epos4Control epos4(nh, private_nh);
    // ROS_INFO("num of moters is : %d");

    ros::Rate sleep_rate(100);

    epos4.get_num_of_motor();


    int step=0;
    int step_num=100;
    // while (ros::ok()) {
        
        
    //     epos4.test_for_motor(step, step_num);
    //     // epos4.read_all();
    //     // cout << step << endl;

        
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    //     step++;
    // }

    ros::spin();

    // epos4.return_to_zero();

    ROS_INFO("The End !");

    return 0;

}