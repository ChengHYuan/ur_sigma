#include <math.h>
#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace std;
using namespace KDL;

class SurgicalRobotUR {
public:
    Tree ur10e_tree;
    Chain ur10e_chain;
    int num_of_joints;
    JntArray q;
    JntArray q_max;
    JntArray q_min;
    Frame pose_end;
    ChainFkSolverPos_recursive *fwdkin;
public:
    SurgicalRobotUR();
};

//构造函数
SurgicalRobotUR::SurgicalRobotUR() {
    this->ur10e_chain;
    // DH（a alpha d theta）
    ur10e_chain.addSegment(Segment("ur10e_link_1", Joint(Joint::RotZ), Frame::DH(0, M_PI / 2, 0.1807, 0)));
    ur10e_chain.addSegment(Segment("ur10e_link_2", Joint(Joint::RotZ), Frame::DH(-0.6127, 0, 0, 0)));
    ur10e_chain.addSegment(Segment("ur10e_link_3", Joint(Joint::RotZ), Frame::DH(-0.57155, 0, 0, 0)));
    ur10e_chain.addSegment(Segment("ur10e_link_4", Joint(Joint::RotZ), Frame::DH(0, M_PI / 2, 0.17415, 0)));
    ur10e_chain.addSegment(Segment("ur10e_link_5", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.11985, 0)));
    ur10e_chain.addSegment(Segment("ur10e_link_6", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.22832, 0)));
    ur10e_chain.addSegment(Segment("endowrist_link1", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.44711, 0)));
    ur10e_chain.addSegment(Segment("endowrist_link2", Joint(Joint::RotZ), Frame::DH(0.013, M_PI / 2, 0, 0)));
    ur10e_chain.addSegment(Segment("endowrist_link3", Joint(Joint::RotZ), Frame::DH(0.025, 0, 0, 0)));
    this->ur10e_tree;
    ur10e_tree = Tree("ur10e_link_0");
    ur10e_tree.addChain(ur10e_chain, "ur10e_link_0");
    //ROS_INFO("chain:%s", kuka_chain.segments[0].getName().c_str());
    //创建关节角度数组
    this->num_of_joints = ur10e_chain.getNrOfJoints();

    this->q;
    q.resize(num_of_joints);
    q.data << 0.2618, -1.39626, -2.2689, 0.523598, 1.30899, 0.0, 1.57079, -1.57079, 0;
    

    ROS_INFO("q:%d",q.data.size());
    double q_0 = 0;

    //创建关节角限制
    // q_max(num_of_joints);
    // q_min(num_of_joints);
    // q_max.data << 170, 120, 170, 120, 170, 120, 175, 180, 60, 90;
    // q_min.data << -170, -120, -170, -120, -170, -120, -175, -180, -60, -90;

    // //获取link数
    // int link_num =kuka_tree.getNrOfSegments();
    // map<string,TreeElement> names_of_seg=kuka_tree.getSegments();

    // ROS_INFO("number of segments is %d",link_num);

    // 创建正运动学链,获取末端位姿，即pose_end
    fwdkin = new ChainFkSolverPos_recursive(ur10e_chain);
    this->pose_end;
    if (fwdkin->JntToCart(q, pose_end, 8) < 0) {
        ROS_INFO("failed to get end pose");
    }
    ROS_INFO("x:%f,y:%f,z:%f", pose_end.p.x(), pose_end.p.y(), pose_end.p.z());
    // cout << "pose: " << pose_end.M.data[0] << " " << pose_end.M.data[1] << " " << pose_end.M.data[2] << endl;
    // cout << pose_end.M.data[3] << " " << pose_end.M.data[4] << " " << pose_end.M.data[5] << " " << endl;
    // cout << pose_end.M.data[6] << " " << pose_end.M.data[7] << " " << pose_end.M.data[8] << " " << endl;
    // cout <<"orientation:"<<pose_end.M.data[7] << endl;

}