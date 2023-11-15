#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "surgical_robot_ur.hpp"
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "printer.hpp"
#include "surgical_robot_pub_sub.hpp"
#include <iiwa_msgs/CartesianPose.h>

using namespace std;

SurgicalRobotUR robot;
Chain robot_chain = robot.ur10e_chain;
ChainFkSolverPos_recursive robot_fk_solver = *robot.fwdkin;
Frame frame_t;

void calcu_msg(KDL::Frame& frame_t, geometry_msgs::PoseStamped& msg) {//发布计算后末端位姿
    double q_x, q_y, q_z, q_w;
    frame_t.M.GetQuaternion(q_x, q_y, q_z, q_w);
    msg.pose.position.x = frame_t.p.x();
    msg.pose.position.y = frame_t.p.y();
    msg.pose.position.z = frame_t.p.z();
    msg.pose.orientation.x = q_x;
    msg.pose.orientation.y = q_y;
    msg.pose.orientation.z = q_z;
    msg.pose.orientation.w = q_w;
    // cout << msg << endl;
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "robot_thans_to_end");
    RobotSubPub sub_pub;
    sub_pub.sub_pub_init();
    //ros::Subscriber sub_of_catesian = sub_pub.m_h.subscribe("/iiwa/state/CartesianPose", 10,call_back);//获取目标位姿
    ros::Publisher pub_of_robot_end = sub_pub.m_h.advertise<geometry_msgs::PoseStamped>("/iiwa/state/CartesianPose_end", 1);//发布计算后的位姿
    geometry_msgs::PoseStamped msg;
    Printer pri;

    ros::Rate rate(200);
    // int itr_step = 0;

    Vector end_position(0.702, 0, 0.08);
    
    Frame end_pose(end_position);
    
    end_pose.M.DoRotY(M_PI / 2);
    // pri.print_frame(end_pose);


    ros::spinOnce();
    while (ros::ok())
    {
        msg.header.stamp=ros::Time::now();

        robot_fk_solver.JntToCart(sub_pub.joints_now, frame_t,-1);
        // pri.print_joints(sub_pub.joints_now);
        calcu_msg(frame_t, msg);
        // calcu_msg(end_pose, msg);
        // pri.print_frame(frame_t);
        // cout << msg << endl;
        pri.print_frame(frame_t);
        pub_of_robot_end.publish(msg);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;

}

