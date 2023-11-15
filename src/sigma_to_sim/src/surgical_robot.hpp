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

class SurgicalRobot {
public:
    Tree kuka_tree;
    Chain kuka_chain;
    int num_of_joints;
    JntArray q;
    JntArray q_max;
    JntArray q_min;
    Frame pose_end;
    ChainFkSolverPos_recursive *fwdkin;
public:
    SurgicalRobot();
};

//构造函数
SurgicalRobot::SurgicalRobot() {
    //从urdf文件获取机器人树
    // this->kuka_tree;
    // if (!kdl_parser::treeFromFile("/home/chy/sigma_iiwa_simulation/sigma_to_sim/src/sigma_to_sim/description/iiwa14.urdf", kuka_tree)) {
    //     ROS_ERROR("Failed to get kdl tree");
    // }
    this->kuka_chain;
    // //从树中提取单链
    // kuka_tree.getChain("iiwa14_link_0", "iiwa14_link_6", kuka_chain);

    kuka_chain.addSegment(Segment("iiwa14_link_1", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.36, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_2", Joint(Joint::RotZ), Frame::DH(0, M_PI / 2, 0, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_3", Joint(Joint::RotZ), Frame::DH(0, M_PI / 2, 0.42, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_4", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_5", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.40, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_6", Joint(Joint::RotZ), Frame::DH(0, M_PI / 2, 0, 0)));
    kuka_chain.addSegment(Segment("iiwa14_link_7", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.2631, 0)));
    kuka_chain.addSegment(Segment("endowrist_link1", Joint(Joint::RotZ), Frame::DH(0, -M_PI / 2, 0.44755, 0)));
    kuka_chain.addSegment(Segment("endowrist_link2", Joint(Joint::RotZ), Frame::DH(0.013, M_PI / 2, 0, 0)));
    kuka_chain.addSegment(Segment("endowrist_link3", Joint(Joint::RotZ), Frame::DH(0, 0, 0, 0)));
    this->kuka_tree;
    kuka_tree = Tree("iiwa14_link_0");
    kuka_tree.addChain(kuka_chain, "iiwa14_link_0");
    //ROS_INFO("chain:%s", kuka_chain.segments[0].getName().c_str());
    //创建关节角度数组
    this->num_of_joints = kuka_chain.getNrOfJoints();

    this->q;
    q.resize(num_of_joints);
    q.data << 0, 0.292532, 0, -1.7418, 0, -0.463514, -1.57081, 1.57079, -1.5708, 0;
    

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
    fwdkin = new ChainFkSolverPos_recursive(kuka_chain);
    this->pose_end;
    if (fwdkin->JntToCart(q, pose_end, -1) < 0) {
        ROS_INFO("failed to get end pose");
    }
    ROS_INFO("x:%f,y:%f,z:%f", pose_end.p.x(), pose_end.p.y(), pose_end.p.z());
    // cout << "pose: " << pose_end.M.data[0] << " " << pose_end.M.data[1] << " " << pose_end.M.data[2] << endl;
    // cout << pose_end.M.data[3] << " " << pose_end.M.data[4] << " " << pose_end.M.data[5] << " " << endl;
    // cout << pose_end.M.data[6] << " " << pose_end.M.data[7] << " " << pose_end.M.data[8] << " " << endl;
    // cout <<"orientation:"<<pose_end.M.data[7] << endl;

}