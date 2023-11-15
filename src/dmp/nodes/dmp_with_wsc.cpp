//
// Created by jiy on 2021/10/26.
//
#include "ros/ros.h"
#include "dmp/dmp.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "robotController/robotController.h"
#include "jacobian.h"
#include <fstream>

#include <gmm/GetConstraint.h>

using namespace std;

const int HZ = 125;
const bool UseSim = true;

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle *n;
    n = new ros::NodeHandle();
    ros::Rate rate(HZ);
    ros::Publisher pathPub = n->advertise<nav_msgs::Path>("dmp/trajectory",10, true);
    ros::ServiceClient clientWsc= n->serviceClient<gmm::GetConstraint>("gmm/get_constriant_couple_term");
    ros::Publisher markerPub = n->advertise<visualization_msgs::Marker>("dmp/target_marker",10, true);

    robotController myController(n,UseSim);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    // trajectory information
    std::vector<double> x_0; x_0.push_back(0.5); x_0.push_back(-0.2);

    std::vector<double> goal; goal.push_back(0.44); goal.push_back(-0.38);
//    std::vector<double> goal; goal.push_back(0.8); goal.push_back(-1.2);
//    std::vector<double> goal; goal.push_back(0.6); goal.push_back(-0.38);

    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = goal[0];
    marker.pose.position.y = goal[1];
    marker.pose.position.z = 1.34306;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    markerPub.publish(marker);


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
    myController.moveToFrame(myController.homeJoint,init_frame,6,true);

    // init visualization msgs
    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    // load dmp parameter from xml file
    vector<dmp::DMPData> dmpList;
    string dmp_file = "/home/jiyguo/ur_ws/src/dmp/data/dmp13.xml";
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
    gmm::GetConstraint srvWsc;
    std::vector<double> couple_term(3);
    while(ros::ok() &!at_goal && cur_t < seg_length) {

        srvWsc.request.data.clear();
        for(int i = 0; i<cur_state.positions.size();++i){
            srvWsc.request.data.push_back(cur_state.positions[i]);
        }
        srvWsc.request.data.push_back(frame.p.z());
        if(clientWsc.call(srvWsc)){
            for (int i = 0; i < srvWsc.response.couple_term.size(); ++i) {
                couple_term[i] = srvWsc.response.couple_term[i];
            }
        }
        std::cout<<"srvWsc.request.data: "<< srvWsc.request.data[0] << " , " << srvWsc.request.data[1]<< " , " << srvWsc.request.data[2]<<std::endl;
        std::cout<<"couple_term: "<< couple_term[0] << " , " << couple_term[1]<< " , " << couple_term[2]<<std::endl;
        // generate ont step by dmp
        generateStep(couple_term, dmpList, x_0, cur_t, goal, goal_thresh, tau, total_dt,
                     integrate_iter, f, cur_state, plan, at_goal);

        // control the robot to planned frame
        frame.p.data[0] = plan.positions[0];
        frame.p.data[1] = plan.positions[1];
        GeometryJacobian::JntToJacobianRight(joint,jac);
        v_ee.vel.x(plan.velocities[0]);
        v_ee.vel.y(plan.velocities[1]);
        GeometryJacobian::jac_inv_multiply_twist(GeometryJacobian::pinv(jac), v_ee, jnt_speed);
        myController.moveToFrame(frame,jnt_speed,total_dt,false);
//        myController.moveToFrame(frame,total_dt, true);
        // update state
        cur_state = plan;
        cur_t += total_dt;
        // publish trajectory
        myController.getRobotFrame(frame);
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
