//
// Created by jiyguo on 2022/3/23.
//
#include "my_dmp.h"
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "robotController/robotController.h"
using namespace myDmp;
using namespace std;
const int HZ = 125;

int main(int argc, char **argv){

    // init
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle *n;
    n = new ros::NodeHandle();
    ros::Rate rate(HZ);
    ros::Publisher pathPub = n->advertise<nav_msgs::Path>("dmp/trajectory",10, true);
    robotController myController(n);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    string dmp_file = "/home/jiyguo/ur_ws/src/dmp/data/dmp02.xml";

    // trajectory information
    DVector x_0; x_0.push_back(0.52); x_0.push_back(-0.22);
    DVector goal; goal.push_back(0.44); goal.push_back(-0.38);
    DVector x_dot_0; x_dot_0.push_back(0.0); x_dot_0.push_back(0.0);
    DVector goal_thresh; goal_thresh.push_back(0.01); goal_thresh.push_back(0.01);
    double seg_length = 20;
    double dt = 1.0 / HZ;
    double cur_t = 0;
    int num_bases = 15;
    MyDmp my_dmp( x_0, goal, x_dot_0, goal_thresh, num_bases,dt);
    my_dmp.Init(dmp_file);

    DMPTraj traj;
//    my_dmp.get_plan(traj,cur_t);
    DMPPoint cur_state;cur_state.positions = x_0;cur_state.velocities = x_dot_0;
    DMPPoint plan_state;
    DVector couple_term(3,0);
    double t_now = 0;
    while(!my_dmp.get_step(cur_state,couple_term,t_now,plan_state)){
        traj.points.push_back(cur_state);
        traj.times.push_back(t_now);
        t_now += dt;
        cur_state = plan_state;
    }

    // init visualization msgs
    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    // move to start point
    KDL::JntArray joint;
    joint.resize(6);
    KDL::Frame init_frame;
    myController.FKine(myController.homeJoint,init_frame);
    init_frame.M = KDL::Rotation::RotX(M_PI);
    init_frame.p.x(x_0[0] );
    init_frame.p.y(x_0[1] );
    myController.moveToFrame(myController.homeJoint,init_frame,3,true);
    KDL::Frame frame = init_frame;
    KDL::Jacobian jac(6);
    KDL::Twist v_ee; v_ee.Zero();
    KDL::JntArray jnt_speed(6);
    myController.getRobotJoints(joint);
    for (int i = 0; i < traj.points.size(); ++i) {
        // control the robot to planned frame
        frame.p.data[0] = traj.points[i].positions[0];
        frame.p.data[1] = traj.points[i].positions[1];
        myController.moveToFrame(frame,dt, false);
        // publish trajectory
        path.header.stamp = ros::Time::now();
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.position.x = frame.p.x();
        this_pose_stamped.pose.position.y = frame.p.y();
        this_pose_stamped.pose.position.z = frame.p.z();
        path.poses.push_back(this_pose_stamped);
        pathPub.publish(path);
        rate.sleep();
    }

}

