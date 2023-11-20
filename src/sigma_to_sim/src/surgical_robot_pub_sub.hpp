#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <string>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>



using namespace std;

//创建机器人发布者订阅者类

class RobotSubPub {

public:
    ros::NodeHandle m_h;
    ros::Subscriber m_sub_frame;
    ros::Subscriber m_sub_joints;
    ros::Subscriber m_sub_joints_real;
    ros::Subscriber m_sub_endo;
    ros::Subscriber m_sub_gripper;
    ros::Subscriber m_sub_of_v_real;

    ros::Publisher m_pub_of_endo;
    ros::Publisher m_pub_of_joints;
    ros::Publisher m_pub_of_joints_real;

    ros::Publisher m_pub_of_joints_vel;
    ros::Publisher m_pub_of_endo_vel;
    ros::Publisher m_pub_of_v;
    ros::Publisher m_pub_of_v_real;

    bool limits_flag = false;

    sensor_msgs::JointState m_jp;
    sensor_msgs::JointState m_jv;

    sensor_msgs::JointState r_jv;
    sensor_msgs::JointState r_jp;


    int hz;
    int base_N;
    int total_N;

    vector<double> joints_limits_max;
    
    ros::Time time;


    KDL::Frame command_frame;
    KDL::JntArray joints_now;
    KDL::JntArray joints_now_real;
    
    vector<KDL::JntArray> joints_save;
    KDL::JntArray joints_velocity;

    bool is_real_get;
    bool is_vrep_get;
    
    vector<double> gap_fri;


    Eigen::VectorXd joints_before;
    Eigen::VectorXd joints_delta;

    KDL::JntArray joints_pubed;



    double gripper_angle;

    RobotSubPub() {

        m_h;
        m_sub_frame;
        m_pub_of_endo;
        m_pub_of_joints;
        m_pub_of_joints_vel;
        m_pub_of_endo_vel;
        m_pub_of_v;

        command_frame;
        m_sub_joints; 
        joints_now;
        gripper_angle;
        time;
        joints_pubed;

        is_real_get = false;
        is_vrep_get = false;

        hz = 200;
        base_N = 6;
        total_N = base_N + 3;

        m_jp.position.resize(base_N);
        m_jv.position.resize(total_N);
        
        joints_now.resize(total_N);
        joints_now_real.resize(total_N);
        joints_before.resize(total_N);
        joints_delta.resize(total_N);
        joints_save.resize(base_N);
        joints_pubed.resize(total_N);

        gap_fri.resize(total_N);

        joints_velocity.resize(total_N);
        KDL::Vector end_position(0.702, 0, 0.08);
    
        command_frame.p = end_position;
        command_frame.M.DoRotY(M_PI / 2);

        for (int i = 0;i < joints_save.size();++i) {
            joints_save[i] = joints_now;
        }
       
        // joints_now.data << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        // joints_limits_max = {170,   120,    170,    120,    170,    120,    175, 120, 60, 60 };//iiwa
        joints_limits_max = { 180,   180,    180,    180,    180,    180,    180, 80, 80 };//iiwa
    };
    void sub_pub_init() {
        m_sub_frame = m_h.subscribe("/iiwa/command/CartesianPose_origin", 1, &RobotSubPub::call_back_frame, this);//获取目标位姿
        m_sub_joints = m_h.subscribe("/left_UR10e/state/JointPosition", 1, &RobotSubPub::call_back_joints, this);//获取仿真机器人角度
        m_sub_joints_real = m_h.subscribe("/ur10e/state/JointState", 1, &RobotSubPub::call_back_joints_real, this);//获取真实机器人角度
        m_sub_endo = m_h.subscribe("/endoWrist/state/JointPosition", 1, &RobotSubPub::call_back_endo, this);//获取当前手术器械角度
        m_sub_gripper = m_h.subscribe("/sigma7/sigma0/gripper_angle", 1, &RobotSubPub::call_back_gripper, this);//获取夹爪角度
        m_sub_of_v_real = m_h.subscribe("/iiwa/state/JointVelocity", 1, &RobotSubPub::call_back_velocity, this);//获取当前iiwa机器人角度

        m_pub_of_endo = m_h.advertise<std_msgs::Float32MultiArray>("/endoWrist/command/JointPosition", 1);//发送手术器械电机角度
        m_pub_of_joints = m_h.advertise<sensor_msgs::JointState>("/left_UR10e/command/JointPosition", 1);//发送iiwa机器人各个关节角度
        m_pub_of_joints_real = m_h.advertise<sensor_msgs::JointState>("/ur10e/command/JointPosition", 1);//发送iiwa机器人各个关节角度
        m_pub_of_v = m_h.advertise<sensor_msgs::JointState>("/left_UR10e/test/Velocity", 1);//发送计算出来的关节角速度
        m_pub_of_v_real = m_h.advertise<sensor_msgs::JointState>("/left_UR10e/state/JointVelocity_2", 1);//发送iiwa实际速度，带时间戳

        // m_pub_of_p_v = m_h.advertise<iiwa_msgs::JointPositionVelocity>("/iiwa/command/JointPositionVelocity", 1);//发送iiwa各个关节的角度和速度

        m_pub_of_joints_vel = m_h.advertise<sensor_msgs::JointState>("/left_UR10e/command/JointVelocity", 1);//发送iiwa机器人各个关节速度
        m_pub_of_endo_vel = m_h.advertise<std_msgs::Float32MultiArray>("/endoWrist/command/JointVelocity", 1);//发送iiwa机器人各个关节速度


        ros::spinOnce();
    }
    
    void pub_of_all(KDL::JntArray& msg1);//发布位置
    void pub_of_all_2(KDL::JntArray& msg1);//发布各个关节速度
    void pub_of_velocity(KDL::JntArray& msg1);//发布iiwa速度,用作观察
    void pub_p_v(sensor_msgs::JointState &msg);//发布iwia关节位置、速度

    void get_joints_velocity(KDL::JntArray& msg1);//从相邻的关节角获取角速度

    void call_back_frame(const geometry_msgs::PoseStamped& msg);
    void call_back_joints(const sensor_msgs::JointState& msg);
    void call_back_joints_real(const sensor_msgs::JointState &msg);
    void call_back_endo(const std_msgs::Float32MultiArray& msg);
    void call_back_gripper(const std_msgs::Float32& msg);
    void call_back_velocity(const sensor_msgs::JointState & msg);
    void check_iiwa_joints(KDL::JntArray& msg1,double& gripper, bool& flag);
    void pub_of_fri(KDL::JntArray& msg1);//仅发布fri关节角,仅在初始化时使用
    KDL::JntArray joint_planning(KDL::JntArray& local, KDL::JntArray& home, int& count, int& count_range);//初始位置的轨迹规划
    KDL::JntArray keep_joints_save(KDL::JntArray& j_in);//发送时通过限制关节角限制安全

};

void RobotSubPub::call_back_frame(const geometry_msgs::PoseStamped& msg) {
    Eigen::Quaterniond quat;
    Eigen::Matrix3d rotate;
    
    // command_frame.M.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    quat.x() = msg.pose.orientation.x;
    quat.y() = msg.pose.orientation.y;
    quat.z() = msg.pose.orientation.z;
    quat.w() = msg.pose.orientation.w;
    rotate = quat.toRotationMatrix();
    KDL::Rotation rotate_M(rotate(0, 0), rotate(0, 1), rotate(0, 2),
                      rotate(1, 0), rotate(1, 1), rotate(1, 2),
                      rotate(2, 0), rotate(2, 1), rotate(2, 2));
    command_frame.M = rotate_M;
    command_frame.p.data[0] = msg.pose.position.x;
    command_frame.p.data[1] = msg.pose.position.y;
    command_frame.p.data[2] = msg.pose.position.z;
    
}

//关节回调
void RobotSubPub::call_back_joints(const sensor_msgs::JointState& msg) {
    is_vrep_get = true;
    time = msg.header.stamp;
    joints_now.data[0] = msg.position[0];
    joints_now.data[1] = msg.position[1];
    joints_now.data[2] = msg.position[2];
    joints_now.data[3] = msg.position[3];
    joints_now.data[4] = msg.position[4];
    joints_now.data[5] = msg.position[5];
    // cout << "j6: " << joints_now.data[5] << endl;
}

void RobotSubPub::call_back_joints_real(const sensor_msgs::JointState &msg){
    is_real_get = true;
    time = msg.header.stamp;
    joints_now_real.data[0] = msg.position[0];
    joints_now_real.data[1] = msg.position[1];
    joints_now_real.data[2] = msg.position[2];
    joints_now_real.data[3] = msg.position[3];
    joints_now_real.data[4] = msg.position[4];
    joints_now_real.data[5] = msg.position[5];
    
}

//手术器械关节角回调
void RobotSubPub::call_back_endo(const std_msgs::Float32MultiArray& msg) {
    joints_now.data[6] = msg.data[0] + M_PI / 2;
    joints_now.data[7] = msg.data[1] - M_PI / 2;
    joints_now.data[8] = msg.data[2];
    joints_now_real.data[6] = msg.data[0] + M_PI / 2;
    joints_now_real.data[7] = msg.data[1] - M_PI / 2;
    joints_now_real.data[8] = msg.data[2];
}

void RobotSubPub::get_joints_velocity(KDL::JntArray& msg1) {}//从相邻的关节角获取角速度

//手术夹角
void RobotSubPub::call_back_gripper(const std_msgs::Float32& msg) {
    gripper_angle = msg.data;
}

// 获取iiwa关节实际速度
void RobotSubPub::call_back_velocity(const sensor_msgs::JointState& msg) {
    r_jv = msg;
}

//机器人需要同时发布关节角和角速度
void RobotSubPub::pub_of_all(KDL::JntArray& msg1) {
    // iiwa_msgs::JointPositionVelocity iiwa_joint;
    std_msgs::Float32MultiArray endo_joints;
    limits_flag = false;
    check_iiwa_joints(msg1,gripper_angle, limits_flag);
    if (!limits_flag) {

        m_jp.header.stamp = ros::Time::now();
        // iiwa_joint.header.stamp = time;

        m_jp.position[0] = msg1.data[0];
        m_jp.position[1] = msg1.data[1];
        m_jp.position[2] = msg1.data[2];
        m_jp.position[3] = msg1.data[3];
        m_jp.position[4] = msg1.data[4];
        m_jp.position[5] = msg1.data[5];


        // cout << "flag4...." << endl;

        // get_joints_velocity(msg1);
        // cout << joints_velocity.data << endl;
        // iiwa_joint.velocity.a1 = joints_velocity.data[0];
        // iiwa_joint.velocity.a2 = joints_velocity.data[1];
        // iiwa_joint.velocity.a3 = joints_velocity.data[2];
        // iiwa_joint.velocity.a4 = joints_velocity.data[3];
        // iiwa_joint.velocity.a5 = joints_velocity.data[4];
        // iiwa_joint.velocity.a6 = joints_velocity.data[5];
        // iiwa_joint.velocity.a7 = joints_velocity.data[6];

        endo_joints.data.push_back(msg1.data[6] - M_PI / 2);
        endo_joints.data.push_back(msg1.data[7] + M_PI / 2);
        endo_joints.data.push_back(msg1.data[8]);
        endo_joints.data.push_back(gripper_angle);

        // joints_delta = msg1.data - joints_before;
        
        // cout << "joints delta are: " << joints_delta << endl;
        m_pub_of_joints.publish(m_jp);
        m_pub_of_joints_real.publish(m_jp);


        // m_pub_of_v_p.publish(iiwa_joint);
        m_pub_of_endo.publish(endo_joints);
        
        // ROS_INFO("messages are sent");
        // joints_before = msg1.data;

        endo_joints.data.clear();
    }
    else {
        ROS_INFO("------------PUB FAILED------------");
    }  
}


void RobotSubPub::pub_of_velocity(KDL::JntArray& msg1) {
    get_joints_velocity(msg1);
    // cout << joints_velocity.data << endl;
    m_jv.header.stamp = ros::Time::now();
    m_jv.velocity[0] = joints_velocity.data[0];
    m_jv.velocity[1] = joints_velocity.data[1];
    m_jv.velocity[2] = joints_velocity.data[2];
    m_jv.velocity[3] = joints_velocity.data[3];
    m_jv.velocity[4] = joints_velocity.data[4];
    m_jv.velocity[5] = joints_velocity.data[5];
    m_pub_of_v.publish(m_jv);
}

// void RobotSubPub::pub_p_v(iiwa_msgs::JointPositionVelocity& msg) {
//     msg.header.stamp = ros::Time::now();
//     m_pub_of_p_v.publish(msg);
// }
//关节角度限制

void RobotSubPub::check_iiwa_joints(KDL::JntArray& msg1,double& gripper, bool& flag) {
    for (int i = 0;i < msg1.data.size() - 3;++i) {
        if (msg1.data[i] > joints_limits_max[i] * M_PI / 180 || msg1.data[i] < -joints_limits_max[i] * M_PI / 180) {
            ROS_WARN("joints %d is out of limits range", i+1);
            flag = true;
        }
    }

    if (msg1.data[6] - M_PI / 2 > joints_limits_max[6] * M_PI / 180 || msg1.data[6] - M_PI / 2 < -joints_limits_max[6] * M_PI / 180) {
        ROS_WARN("joints %d is out of limits range", 7);
        flag = true;
    }
    
    if (msg1.data[7] + M_PI / 2 > joints_limits_max[7] * M_PI / 180 || msg1.data[7] + M_PI / 2 < -joints_limits_max[7] * M_PI / 180) {
        ROS_WARN("joints %d is out of limits range", 8);
        
        flag = true;
    }
    if (msg1.data[8] + gripper / 2 > joints_limits_max[8] * M_PI / 180 || msg1.data[8] + gripper / 2 < -joints_limits_max[8] * M_PI / 180 ||
        msg1.data[8]-gripper/2> joints_limits_max[8] * M_PI / 180 || msg1.data[8]-gripper/2< -joints_limits_max[8] * M_PI / 180) {
        ROS_WARN("joints %d is out of limits range", 9);
        flag = true;
    }
}

KDL::JntArray RobotSubPub::joint_planning(KDL::JntArray& local, KDL::JntArray& home, int& count,int& count_range) {
    KDL::JntArray answer(total_N);
    
    if (gap_fri[0] == 0.0) {
        for (int i = 0;i < total_N;++i) {
            gap_fri[i] = home.data[i] - local.data[i];
        }
    }
    for (int i = 0;i < total_N;++i) {
        answer.data[i] = local.data[i] + (gap_fri[i] * count) / count_range;
    }
    return answer;
}

KDL::JntArray RobotSubPub::keep_joints_save(KDL::JntArray& j_in) {//发送时通过限制关节角限制安全
    KDL::JntArray jin = j_in;
    KDL::JntArray jout(jin.data.size());
    vector<double> j_bonus(jin.data.size());
    vector<int> over_flags = vector<int>(9, 0);
    bool is_save_active=false;
    double max_velocity = 0.5;//单位rad/s;
    
    double max_gap = 1.0 / hz * max_velocity;
    // ROS_INFO("max gap: %f", max_gap);
    for (int i = 0;i < jin.data.size();++i) {
        if (jin.data[i]-joints_pubed.data[i]>max_gap) {
            over_flags[i] = 1;
            is_save_active = true;
        }
        else if (jin.data[i] - joints_pubed.data[i] < -max_gap) {
            over_flags[i] = -1;
            is_save_active = true;
        }
        else {

        }
    }

    if (is_save_active) {//超出时，进行插值，防止相邻角度差距太大
        for (int i = 0;i < jin.data.size();++i) {
            if (over_flags[i]==1) {
                jout.data[i] = joints_pubed.data[i] + max_gap;
            }
            else if (over_flags[i] == -1) {
                jout.data[i] = joints_pubed.data[i] - max_gap;
            }
            else {
                jout.data[i] = jin.data[i];
            }
        }
    }
    else {
        jout = jin;
    }
    joints_pubed = jout;
    return jout;
}
