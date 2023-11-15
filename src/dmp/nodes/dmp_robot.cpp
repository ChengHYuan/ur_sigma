//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "robotController/robotController.h"
#include "jacobian.h"
#include "tinyxml2.h"

#include <fstream>

using namespace std;

const int HZ = 125;

int main(int argc, char **argv)
{
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

    // trajectory information
    std::vector<double> x_0; x_0.push_back(0.52); x_0.push_back(-0.22);
    std::vector<double> goal; goal.push_back(0.44); goal.push_back(-0.38);
    std::vector<double> x_dot_0; x_dot_0.push_back(0.0); x_dot_0.push_back(0.0);
    std::vector<double> goal_thresh; goal_thresh.push_back(0.01); goal_thresh.push_back(0.01);
    double seg_length = 20;
    double total_dt = 1.0 / HZ;
    int integrate_iter = 1;
    double cur_t = 0;
    dmp::DMPPoint cur_state; cur_state.positions = x_0; cur_state.velocities = x_dot_0;
    dmp::DMPPoint plan;
    bool at_goal = false;
    int dims = x_0.size();

    // move to start point
    KDL::JntArray joint;
    joint.resize(6);
    KDL::Frame init_frame;
    myController.FKine(myController.homeJoint,init_frame);
    init_frame.M = KDL::Rotation::RotX(M_PI);
    init_frame.p.x(x_0[0] );
    init_frame.p.y(x_0[1] );
    myController.moveToFrame(myController.homeJoint,init_frame,3,true);

    // init visualization msgs
    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    // load dmp parameter from xml file
    vector<dmp::DMPData> dmpList;
    string dmp_file = "/home/jiyguo/ur_ws/src/dmp/data/dmp2.xml";
    double tau;
    if(!dmp::load_dmp_para(dmp_file, dmpList, tau)){
        return -1;
    }

    // construct the force term function by dmp parameters
    dmp::FunctionApprox **f = new dmp::FunctionApprox*[dims];
    for(int i=0; i<dims; i++){
        double base_width = pow(dmpList[i].weights.size(),1.5) * dmp::alpha;
        f[i] = new dmp::RadialApprox(dmpList[i].weights, base_width, dmp::alpha);
    }

    // loop
    dmp::DMPPointStamp cur;
    KDL::Frame frame = init_frame;
    KDL::Jacobian jac(6);
    KDL::Twist v_ee; v_ee.Zero();
    KDL::JntArray jnt_speed(6);
    myController.getRobotJoints(joint);
    while(ros::ok() &!at_goal && cur_t < seg_length) {
        // generate ont step by dmp
        generateStep(dmpList, x_0, cur_t, goal, goal_thresh, tau, total_dt,
                     integrate_iter, f, cur_state, plan, at_goal);

        // control the robot to planned frame
        frame.p.data[0] = plan.positions[0];
        frame.p.data[1] = plan.positions[1];
        GeometryJacobian::JntToJacobianRight(joint,jac);
        v_ee.vel.x(plan.velocities[0]);
        v_ee.vel.y(plan.velocities[1]);
        GeometryJacobian::jac_inv_multiply_twist(GeometryJacobian::pinv(jac), v_ee, jnt_speed);
        myController.moveToFrame(frame,jnt_speed,total_dt, false);
//        myController.moveToFrame(frame,total_dt, true);

        // update state
        cur_state = plan;
        cur_t += total_dt;

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

    //Clean up
    for(int i=0; i<dims; i++){
        delete f[i];
    }
    delete[] f;

    return 0;
}
