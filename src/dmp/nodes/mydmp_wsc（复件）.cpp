//
// Created by jiyguo on 2022/4/7.
//

#include "ros/ros.h"
#include "dmp/DMPPointStamp.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "robotController/robotController.h"
#include "jacobian.h"
#include <fstream>
#include "dmp/DMPPoint.h"
#include "Eigen/Core"
#include "my_dmp.h"

#include <gmm/GetConstraint.h>

using namespace std;

typedef std::vector<double> DVector;

const int HZ = 125;
const bool UseSim = true;
const int N_BASES = 15;
const double DT = 1.0/HZ;

// trajectory information
const string dmp_file = "/home/jiyguo/ur_ws/src/dmp/data/dmp01.xml";
//const DVector x_0 = {-0.5 , -0.2};
//const DVector goal = {-0.44 , -0.38};
const DVector x_0 = {-0.57 , -0.2};
const DVector goal = {-0.64 , -0.38};
//const DVector goal = {0.72 , -1.12};
const DVector x_dot_0 = {0.0 , 0.0};
const DVector goal_thresh = {0.005 , 0.005};
const double seg_length = 20;

ros::Publisher markerPub;
void marker_pub(const DVector& place);

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "dmp_ent");
    ros::NodeHandle *n;
    n = new ros::NodeHandle();
    ros::Rate rate(HZ);
    ros::Publisher pathPub = n->advertise<nav_msgs::Path>("dmp/trajectory",10, true);
    ros::ServiceClient clientWsc= n->serviceClient<gmm::GetConstraint>("gmm/get_constriant_couple_term");

    markerPub = n->advertise<visualization_msgs::Marker>("dmp/target_marker",10, true);
    marker_pub(goal);

    robotController myController(n,UseSim);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Duration(1).sleep();

    double cur_t = 0;
    dmp::DMPPoint cur_state; cur_state.positions = x_0; cur_state.velocities = x_dot_0;
    dmp::DMPPoint plan;
    bool at_goal = false;

    // move to start point
    KDL::JntArray joint;
    joint.resize(6);
    KDL::Frame init_frame;
    myController.FKine(myController.homeJoint,init_frame);
    init_frame.M = KDL::Rotation::RotX(M_PI);
    init_frame.p.x(x_0[0] );
    init_frame.p.y(x_0[1] );
//    myController.moveToFrame(myController.homeJoint,init_frame,6,true);
//    myController.moveToFrame(init_frame,2,true);

    // init visualization msgs
    nav_msgs::Path path;
    path.header.frame_id="world";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.header.frame_id = "world";

    auto my_dmp = new myDmp::MyDmp( x_0, goal, x_dot_0, goal_thresh, N_BASES, DT);
//    string dmp_file = "/home/jiyguo/ur_ws/src/dmp/data/dmp13.xml";
    my_dmp->Init(dmp_file);

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
        if(my_dmp->get_step(cur_state,couple_term,cur_t,plan)){
            break;
        }
//        if(my_dmp->get_step(cur_state,cur_t,plan)){
//            break;
//        }

        // control the robot to planned frame
        frame.p.data[0] = plan.positions[0];
        frame.p.data[1] = plan.positions[1];
        GeometryJacobian::JntToJacobianRight(joint,jac);
        v_ee.vel.x(plan.velocities[0]);
        v_ee.vel.y(plan.velocities[1]);
        GeometryJacobian::jac_inv_multiply_twist(GeometryJacobian::pinv(jac), v_ee, jnt_speed);
//        myController.moveToFrame(frame,jnt_speed,DT,false);
//        myController.moveToFrame(frame,total_dt, true);
        // update state
        cur_state = plan;
        cur_t += DT;
        // publish trajectory
//        myController.getRobotFrame(frame);
        path.header.stamp = ros::Time::now();
        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.pose.position.x = frame.p.x();
        this_pose_stamped.pose.position.y = frame.p.y();
        this_pose_stamped.pose.position.z = frame.p.z();
        path.poses.push_back(this_pose_stamped);
        pathPub.publish(path);
        rate.sleep();
    }
    delete my_dmp;
    return 0;
}


void marker_pub(const DVector& place){
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = place[0];
    marker.pose.position.y = place[1];
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
}


